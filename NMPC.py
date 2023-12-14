# python libraries
import casadi as ca
import numpy as np

# import dubins
from dubins_path import optimized_dubins_path


class NMPC:
    def __init__(self) -> None:
        self.L = 2.5  # Car length (used in car dynamics)
        self.dt = 0.01  # Time step
        self.N = 10  # Prediction horizon
        self.Q = np.diag([1, 1, 1])
        self.R = np.diag([0.005, 0.005])

    # Define the Runge-Kutta method
    def rk4_step(self, f, x, u, h):
        k1 = f(x, u)
        k2 = f(x + h / 2 * k1, u)
        k3 = f(x + h / 2 * k2, u)
        k4 = f(x + h * k3, u)
        x_next = x + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
        return x_next

    def car_dynamics(self, x, u):
        # Car dynamics model (simplified bicycle model)
        # x = [x, y, θ, v] = [x position, y position, orientation (heading angle), speed of the vehicle]
        # u = [a, δ] = [acceleration, steering angle]
        x_dot = x[3] * ca.cos(x[2])
        y_dot = x[3] * ca.cos(x[2])
        theta_dot = x[3] * ca.sin(u[1]) / self.L
        v_dot = u[0]

        return [x_dot, y_dot, theta_dot, v_dot]

    def agv_dynamics(self, x, u):
        # AGV dynamics model
        # x = [x position, y position, heading]
        # u = [velocity, steering/angular velocity]
        x_dot = u[0] * ca.cos(x[2])
        y_dot = u[0] * ca.sin(x[2])
        theta_dot = u[1]
        return [x_dot, y_dot, theta_dot]

    def solve_nmpc(
        self, x_initial, y_initial, theta_initial, x_final, y_final, theta_final
    ):
        initial_state = [
            x_initial,
            y_initial,
            theta_initial,
        ]  # Initial state [x, y, theta]
        end_pose = [x_final, y_final, theta_final]  # End pose [x, y, theta]

        # Apply optimized dubins path for initial guess:
        dubins_xx, dubins_yy, dubins_yaws = optimized_dubins_path(
            initial_state, end_pose, 0.05
        )

        print("dubins length: ", len(dubins_xx))

        # Create an optimization problem
        opti = ca.Opti()

        # Variables
        X = opti.variable(3, self.N + 1)  # State trajectory [x, y, theta]
        U = opti.variable(2, self.N)  # Control trajectory [velocity, steering angle]

        # Initial state
        opti.subject_to(X[:, 0] == initial_state)

        # System dynamics constraints
        for i in range(self.N):
            # # Euler's method
            # x_next = X[:, i] + self.dt * ca.vertcat(*self.agv_dynamics(X[:, i], U[:, i]))

            # RK4 integration
            k1 = self.dt * ca.vertcat(*self.agv_dynamics(X[:, i], U[:, i]))
            k2 = self.dt * ca.vertcat(*self.agv_dynamics(X[:, i] + 0.5 * k1, U[:, i]))
            k3 = self.dt * ca.vertcat(*self.agv_dynamics(X[:, i] + 0.5 * k2, U[:, i]))
            k4 = self.dt * ca.vertcat(*self.agv_dynamics(X[:, i] + k3, U[:, i]))
            x_next = X[:, i] + (k1 + 2 * k2 + 2 * k3 + k4) / 6.0

            opti.subject_to(X[:, i + 1] == x_next)

        # Objective function
        obj = 0
        for i in range(self.N):
            # obj += U[:, i].T @ self.R @ U[:, i]
            if i < len(dubins_xx):
                error = X[:, i] - ca.vertcat(dubins_xx[i], dubins_yy[i], dubins_yaws[i])
                obj += error.T @ self.Q @ error

        opti.minimize(obj)

        if len(dubins_xx) <= self.N:
            # # Create an initial guess for the 'x' and 'y' components as a straight line
            x_guess = np.linspace(initial_state[0], end_pose[0], self.N + 1)
            y_guess = np.linspace(initial_state[1], end_pose[1], self.N + 1)

            # Convert the initial guess to CasADi's DM type
            X0_init = np.vstack(
                [x_guess, y_guess, initial_state[2] * np.ones(self.N + 1)]
            )  # For a straight line
        else:
            X0_init = np.vstack(
                [
                    dubins_xx[0 : self.N + 1],
                    dubins_yy[0 : self.N + 1],
                    dubins_yaws[0 : self.N + 1],
                ]
            )

        # Set the initial guess for the control trajectory
        U0_init = np.zeros((2, self.N))

        opti.set_initial(X, X0_init)
        opti.set_initial(U, U0_init)

        # For QP method:
        opts = dict()
        # opts["qpsol"] = "qrqp"
        # opts["print_header"] = False
        # opts["print_iteration"] = False
        # opts["print_time"] = False
        # opts["qpsol_options"] = {}
        # opts["qpsol_options"]["print_iter"] = False
        # opts["qpsol_options"]["print_header"] = False
        # opts["qpsol_options"]["print_info"] = False
        # opts["qpsol_options"]["error_on_fail"] = False
        # opts = {
        #     "qpsol": "qrqp",  # Use the SQP solver
        #     "print_time": 0,  # Do not print solver details
        #     "verbose_init": 0,  # Do not print initial solver status
        #     "max_iter": 1000,  # Maximum number of iterations for SQP solver
        #     # "max_time": 1.0,  # Maximum allowed time for the solver in seconds
        #     # "print_level": 0,  # Suppress solver output
        #     # Add other solver options as needed
        # }
        # opti.solver("sqpmethod", opts)

        # QRSQP

        # IPOPT
        opts = {
            "ipopt.print_level": 5,  # To disable solver printing
            "print_time": 0,  # To disable printing the solve time
            "ipopt.tol": 1e-4,  # Adjust the tolerance for convergence
            "ipopt.max_iter": 20000,  # Increase the maximum number of iterations
        }

        opti.solver("ipopt", opts)

        # Solve the NMPC problem
        sol = opti.solve()  # actual solve

        # # Extract the first optimal control sequence
        # x_opt = sol.value(X)

        # print("sol: ", sol.value(X))
        # print("sol shape: ", sol.value(X).shape)
        # print("xx: ", sol.value(X)[0, :])
        # print("yy: ", sol.value(X)[1, :])

        return sol.value(X)[0, :], sol.value(X)[1, :], sol.value(X)[2, :]


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
