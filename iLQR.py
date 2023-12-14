# python libraries
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
import math

# pydrake imports
from pydrake.all import (
    Variable,
    SymbolicVectorSystem,
    DiagramBuilder,
    LogVectorOutput,
    Simulator,
    ConstantVectorSource,
    MathematicalProgram,
    Solve,
    SnoptSolver,
    PiecewisePolynomial,
)
import pydrake.symbolic as sym


class derivatives:
    def __init__(self, discrete_dynamics, cost_stage, cost_final, n_x, n_u):
        self.x_sym = np.array([sym.Variable("x_{}".format(i)) for i in range(n_x)])
        self.u_sym = np.array([sym.Variable("u_{}".format(i)) for i in range(n_u)])
        x = self.x_sym
        u = self.u_sym

        l = cost_stage(x, u)
        self.l_x = sym.Jacobian([l], x).ravel()
        self.l_u = sym.Jacobian([l], u).ravel()
        self.l_xx = sym.Jacobian(self.l_x, x)
        self.l_ux = sym.Jacobian(self.l_u, x)
        self.l_uu = sym.Jacobian(self.l_u, u)

        l_final = cost_final(x)
        self.l_final_x = sym.Jacobian([l_final], x).ravel()
        self.l_final_xx = sym.Jacobian(self.l_final_x, x)

        f = discrete_dynamics(x, u)
        self.f_x = sym.Jacobian(f, x)
        self.f_u = sym.Jacobian(f, u)

    def stage(self, x, u):
        env = {self.x_sym[i]: x[i] for i in range(x.shape[0])}
        env.update({self.u_sym[i]: u[i] for i in range(u.shape[0])})

        l_x = sym.Evaluate(self.l_x, env).ravel()
        l_u = sym.Evaluate(self.l_u, env).ravel()
        l_xx = sym.Evaluate(self.l_xx, env)
        l_ux = sym.Evaluate(self.l_ux, env)
        l_uu = sym.Evaluate(self.l_uu, env)

        f_x = sym.Evaluate(self.f_x, env)
        f_u = sym.Evaluate(self.f_u, env)

        return l_x, l_u, l_xx, l_ux, l_uu, f_x, f_u

    def final(self, x):
        env = {self.x_sym[i]: x[i] for i in range(x.shape[0])}

        l_final_x = sym.Evaluate(self.l_final_x, env).ravel()
        l_final_xx = sym.Evaluate(self.l_final_xx, env)

        return l_final_x, l_final_xx


class ILQR:
    def __init__(self, robot="amr") -> None:
        self.robot = robot
        self.setup_problem()
        self.r = 2.0
        self.v_target = 0.5
        self.eps = 1e-6  # The derivative of sqrt(x) at x=0 is undefined. Avoid by subtle smoothing
        self.goal = (0, -2, 0)  # x, y, yaw
        self.N = 50
        self.u_guess = np.random.randn(self.N - 1, self.n_u) * 0.0001

    def setup_problem(self):
        if self.robot is "amr":
            self.n_x = 3
            self.n_u = 2
            self.mapped_dynamics = self.amr_dynamics
        elif self.robot is "car":
            self.n_x = 5
            self.n_u = 2
            self.mapped_dynamics = self.car_continuous_dynamics

    def get_x0(self, x_initial, y_initial, theta_initial):
        if self.robot is "amr":
            return np.array([x_initial, y_initial, theta_initial])
        elif self.robot is "car":
            return np.array([x_initial, y_initial, theta_initial, 0.0, 0.0])

    def car_continuous_dynamics(self, x, u):
        # x = [x position, y position, heading, speed, steering angle]
        # u = [acceleration, steering velocity]
        # m = sym if x.dtype == object else np  # Check type for autodiff
        heading = x[2]
        v = x[3]
        steer = x[4]
        x_d = np.array(
            [v * np.cos(heading), v * np.sin(heading), v * np.tan(steer), u[0], u[1]]
        )
        return x_d

    def amr_dynamics(self, x, u):
        # x = [x position, y position, heading]
        # u = [linear velocity, angular velocity]
        heading = x[2]
        v = u[0]
        w = u[1]
        x_d = np.array([v * np.cos(heading), v * np.sin(heading), w])
        return x_d

    # Define the Runge-Kutta method
    def rk4_step(self, f, x, u, h):
        k1 = f(x, u)
        k2 = f(x + h / 2 * k1, u)
        k3 = f(x + h / 2 * k2, u)
        k4 = f(x + h * k3, u)
        x_next = x + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
        return x_next

    # Used Euler integration
    def discrete_dynamics(self, x, u):
        # Euler integration
        # dt = 0.1
        # x_next = x + dt * self.mapped_dynamics(x, u)

        return self.rk4_step(self.mapped_dynamics, x, u, 0.1)

    # Given an initial state x0 and a guess of a control state u_trj[0:N-1]
    # we rolloat the state trajectory x[0:N] until the time horizon N
    def rollout(self, x0, u_trj):
        # x_trj = np.zeros((u_trj.shape[0] + 1, x0.shape[0]))
        N = u_trj.shape[0]
        x_trj = np.array([x0])
        x_next = x0
        for i in range(0, N):
            x_next = self.discrete_dynamics(
                x_next, np.array([u_trj[i][0], u_trj[i][1]])
            )
            x_trj = np.append(x_trj, [x_next], axis=0)

        return x_trj

    # We define the stage cost function, and final cost function ,
    # The goal of these cost functions is to drive the vehicle along a circle with radius
    # r around the origin with a desired speed.
    def cost_stage(self, x, u):
        m = sym if x.dtype == object else np  # Check type for autodiff
        c_goal = m.sqrt((self.goal[0] - x[0]) ** 2 + (self.goal[1] - x[1]) ** 2)
        c_heading = (x[2] - self.goal[2]) ** 2
        # c_circle = (m.sqrt(x[0] ** 2 + x[1] ** 2 + self.eps) - self.r) ** 2
        c_speed = (x[0] - self.v_target) ** 2 * 0.01
        c_control = (u[0] ** 2 + u[1] ** 2) * 0.1
        return c_goal + c_control + c_heading + c_speed
        return c_goal + c_speed + c_control + c_heading  # + c_circle

    def cost_final(self, xf):
        m = sym if xf.dtype == object else np  # Check type for autodiff
        c_goal = m.sqrt((self.goal[0] - xf[0]) ** 2 + (self.goal[1] - xf[1]) ** 2)
        c_heading = (xf[2] - self.goal[2]) ** 2
        # c_circle = (m.sqrt(xf[0] ** 2 + xf[1] ** 2 + self.eps) - self.r) ** 2
        c_speed = (xf[0] - self.v_target) ** 2 * 0.01
        return c_goal + c_heading + c_speed  # + c_circle + c_speed

    def cost_trj(self, x_trj, u_trj):
        N = u_trj.shape[0]
        total = 0.0
        # Stage Cost
        for i in range(0, N):
            total += self.cost_stage(x_trj[i], u_trj[i])
        # Final Cost
        total += self.cost_final(x_trj[-1])
        return total

    def Q_terms(self, l_x, l_u, l_xx, l_ux, l_uu, f_x, f_u, V_x, V_xx):
        Q_x = l_x + np.dot(f_x.T, V_x)
        Q_u = l_u + np.dot(f_u.T, V_x)
        Q_xx = l_xx + np.dot(f_x.T, np.dot(V_xx, f_x))
        Q_ux = l_ux + np.dot(f_u.T, np.dot(V_xx, f_x))
        Q_uu = l_uu + np.dot(f_u.T, np.dot(V_xx, f_u))
        return Q_x, Q_u, Q_xx, Q_ux, Q_uu

    def gains(self, Q_uu, Q_u, Q_ux):
        Q_uu_inv = np.linalg.inv(Q_uu)
        k = -np.dot(Q_uu_inv, Q_u)
        K = -np.dot(Q_uu_inv, Q_ux)
        return k, K

    # Double check this
    def V_terms(self, Q_x, Q_u, Q_xx, Q_ux, Q_uu, K, k):
        V_x = (
            Q_x + np.dot(Q_ux.T, k) + np.dot(K.T, Q_u) + np.dot(K.T, np.dot(Q_uu.T, k))
        )
        V_xx = (
            Q_xx + np.dot(Q_ux.T, K) + np.dot(K.T, Q_ux) + np.dot(K.T, np.dot(Q_uu, K))
        )
        return V_x, V_xx

    # We can also estimate by how much we expect to reduce the cost by applying the optimal controls
    # The result is implemented below and is a useful aid in checking how accurate the quadratic approximation
    #  is during convergence of iLQR and adapting stepsize and regularization
    def expected_cost_reduction(self, Q_u, Q_uu, k):
        return np.dot(-Q_u.T, k) - 0.5 * np.dot(k.T, np.dot(Q_uu, k))

    # In the forward pass, at each timestep the new updated control u' = u_hat + k + K(x' - x_hat)
    # is applied and the dynamis propagated based on the updated control. The nominal control and state trajectory
    # u_hat and x_hat with which we computed k and K are then updated and we receive a new set of state and control
    #  trajectories.
    def forward_pass(self, x_trj, u_trj, k_trj, K_trj):
        x_trj_new = np.zeros(x_trj.shape)
        x_trj_new[0, :] = x_trj[0, :]
        u_trj_new = np.zeros(u_trj.shape)
        for n in range(u_trj.shape[0]):
            u_trj_new[n, :] = (
                u_trj[n, :]
                + k_trj[n, :]
                + np.dot(K_trj[n, :, :], (x_trj_new[n, :] - x_trj[n, :]))
            )
            x_trj_new[n + 1, :] = self.discrete_dynamics(
                x_trj_new[n, :], u_trj_new[n, :]
            )  # Apply dynamics
        return x_trj_new, u_trj_new

    def backward_pass(self, x_trj, u_trj, regu):
        k_trj = np.zeros([u_trj.shape[0], u_trj.shape[1]])
        K_trj = np.zeros([u_trj.shape[0], u_trj.shape[1], x_trj.shape[1]])
        expected_cost_redu = 0

        # Set terminal boundary condition for value function (V_x, V_xx)
        V_x = np.zeros((x_trj.shape[1],))
        V_xx = np.zeros((x_trj.shape[1], x_trj.shape[1]))

        # Instantiate derivatives class
        derivs = derivatives(
            self.discrete_dynamics, self.cost_stage, self.cost_final, self.n_x, self.n_u
        )

        for n in range(u_trj.shape[0] - 1, -1, -1):
            # Compute derivatives and Q-terms
            l_x, l_u, l_xx, l_ux, l_uu, f_x, f_u = derivs.stage(
                x_trj[n, :], u_trj[n, :]
            )
            Q_x, Q_u, Q_xx, Q_ux, Q_uu = self.Q_terms(
                l_x, l_u, l_xx, l_ux, l_uu, f_x, f_u, V_x, V_xx
            )

            # Add regularization to ensure Q_uu is invertible and nicely conditioned
            Q_uu_regu = Q_uu + np.eye(Q_uu.shape[0]) * regu

            # Calculate gains
            k, K = self.gains(Q_uu_regu, Q_u, Q_ux)
            k_trj[n, :] = k
            K_trj[n, :, :] = K

            # Update value function
            V_x, V_xx = self.V_terms(Q_x, Q_u, Q_xx, Q_ux, Q_uu, K, k)

            expected_cost_redu += self.expected_cost_reduction(Q_u, Q_uu, k)

        return k_trj, K_trj, expected_cost_redu

    def run_ilqr(self, x0, max_iter=50, regu_init=100):
        # First forward rollout
        u_trj = self.u_guess
        x_trj = self.rollout(x0, u_trj)
        total_cost = self.cost_trj(x_trj, u_trj)
        regu = regu_init
        max_regu = 10000
        min_regu = 0.01

        # Setup traces
        cost_trace = [total_cost]
        expected_cost_redu_trace = []
        redu_ratio_trace = [1]
        redu_trace = []
        regu_trace = [regu]

        # Run main loop
        for it in range(max_iter):
            # Backward and forward pass
            k_trj, K_trj, expected_cost_redu = self.backward_pass(x_trj, u_trj, regu)
            x_trj_new, u_trj_new = self.forward_pass(x_trj, u_trj, k_trj, K_trj)
            # Evaluate new trajectory
            total_cost = self.cost_trj(x_trj_new, u_trj_new)
            cost_redu = cost_trace[-1] - total_cost
            redu_ratio = cost_redu / abs(expected_cost_redu)
            # Accept or reject iteration
            if cost_redu > 0:
                # Improvement! Accept new trajectories and lower regularization
                redu_ratio_trace.append(redu_ratio)
                cost_trace.append(total_cost)
                x_trj = x_trj_new
                u_trj = u_trj_new
                regu *= 0.7
            else:
                # Reject new trajectories and increase regularization
                regu *= 2.0
                cost_trace.append(cost_trace[-1])
                redu_ratio_trace.append(0)
            regu = min(max(regu, min_regu), max_regu)
            regu_trace.append(regu)
            redu_trace.append(cost_redu)

            # Early termination if expected improvement is small
            if expected_cost_redu <= 1e-6:
                break

        # Warm start for next call of iLQR
        # self.u_guess = u_trj

        return x_trj, u_trj, cost_trace, regu_trace, redu_ratio_trace, redu_trace

    def solve_ilqr(
        self, x_initial, y_initial, theta_initial, x_final, y_final, theta_final
    ):
        # Setup problem and call iLQR
        x0 = self.get_x0(x_initial, y_initial, theta_initial)
        max_iter = 50
        regu_init = 100

        self.goal = (x_final, y_final, math.radians(theta_final))

        (
            x_trj,
            u_trj,
            cost_trace,
            regu_trace,
            redu_ratio_trace,
            redu_trace,
        ) = self.run_ilqr(x0, max_iter, regu_init)

        print("u_trj: ", u_trj)

        return x_trj[:, 0], x_trj[:, 1], x_trj[:, 2]





# Debug

# N = 10
# x0 = np.array([1, 0, 0, 1, 0])
# u_trj = np.zeros((N - 1, n_u))
# x_trj = rollout(x0, u_trj)

# cost_trj(x_trj, u_trj)

# # Test the output:
# derivs = derivatives(discrete_dynamics, cost_stage, cost_final, n_x, n_u)
# x_d = np.array([0, 0, 0, 0, 0])
# u_d = np.array([0, 0])
# print(derivs.stage(x_d, u_d))
# print(derivs.final(x_d))

# # Setup problem and call iLQR
# x0 = np.array([-3.0, 1.0, -0.2, 0.0, 0.0])
# N = 50
# max_iter = 50
# regu_init = 100
# x_trj, u_trj, cost_trace, regu_trace, redu_ratio_trace, redu_trace = run_ilqr(
#     x0, N, max_iter, regu_init
# )


# plt.figure(figsize=(9.5, 8))
# # Plot circle
# theta = np.linspace(0, 2 * np.pi, 100)
# plt.plot(r * np.cos(theta), r * np.sin(theta), linewidth=5)
# ax = plt.gca()

# # Plot resulting trajecotry of car
# plt.plot(x_trj[:, 0], x_trj[:, 1], linewidth=5)
# w = 2.0
# h = 1.0

# # Plot rectangles
# for n in range(x_trj.shape[0]):
#     rect = mpl.patches.Rectangle((-w / 2, -h / 2), w, h, fill=False)
#     t = (
#         mpl.transforms.Affine2D()
#         .rotate_deg_around(0, 0, np.rad2deg(x_trj[n, 2]))
#         .translate(x_trj[n, 0], x_trj[n, 1])
#         + ax.transData
#     )
#     rect.set_transform(t)
#     ax.add_patch(rect)
# ax.set_aspect(1)
# plt.ylim((-3, 3))
# plt.xlim((-4.5, 3))
# plt.tight_layout()
# plt.subplots(figsize=(10, 6))
# # Plot results
# plt.subplot(2, 2, 1)
# plt.plot(cost_trace)
# plt.xlabel("# Iteration")
# plt.ylabel("Total cost")
# plt.title("Cost trace")

# plt.subplot(2, 2, 2)
# delta_opt = np.array(cost_trace) - cost_trace[-1]
# plt.plot(delta_opt)
# plt.yscale("log")
# plt.xlabel("# Iteration")
# plt.ylabel("Optimality gap")
# plt.title("Convergence plot")

# plt.subplot(2, 2, 3)
# plt.plot(redu_ratio_trace)
# plt.title("Ratio of actual reduction and expected reduction")
# plt.ylabel("Reduction ratio")
# plt.xlabel("# Iteration")

# plt.subplot(2, 2, 4)
# plt.plot(regu_trace)
# plt.title("Regularization trace")
# plt.ylabel("Regularization")
# plt.xlabel("# Iteration")
# plt.tight_layout()
# plt.show()

# print("Finished program")
