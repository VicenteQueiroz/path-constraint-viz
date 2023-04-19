import numpy as np
import math
from pydrake.all import MathematicalProgram, Solve, eq, le, sin, cos, sqrt


def solve_traj_opt_ackerman(
    x_initial, y_initial, theta_initial, x_final, y_final, theta_final
):
    # Create the optimization problem
    prog = MathematicalProgram()

    # Define the robot parameters
    L = 0.5  # wheelbase
    max_v = 1.0  # maximum linear velocity
    max_w = 2.0  # maximum angular velocity
    max_v_steer = 0.5  # maximum steering linear velocity
    delta_max = math.pi / 6  # maximum steering angle

    # Define the Runge-Kutta method
    def rk4_step(f, x, u, h):
        k1 = f(x, u)
        k2 = f(x + h / 2 * k1, u)
        k3 = f(x + h / 2 * k2, u)
        k4 = f(x + h * k3, u)
        x_next = x + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
        return x_next

    # Define the dynamics of the Ackerman drive in a Drake class
    class AckermanDrive:
        def __init__(self, L):
            self.L = L

        def f(self, x, u):
            v = u[0]
            w = u[1]
            theta = x[2]
            x_dot = v * np.cos(theta)
            y_dot = v * np.sin(theta)
            theta_dot = v / self.L * np.tan(w)
            return np.array([x_dot, y_dot, theta_dot])

    # Create an instance of the AckermanDrive class
    ackerman = AckermanDrive(L)

    # Define the number of timesteps and the duration of each timestep
    h = 0.01
    N = int((x_final - x_initial) / h)
    # n_timesteps = 100
    # tf = 10.0
    # dt = tf / n_timesteps

    # Define the optimization variables
    x = prog.NewContinuousVariables(N + 1, "x")
    y = prog.NewContinuousVariables(N + 1, "y")
    theta = prog.NewContinuousVariables(N + 1, "theta")
    # Declare control variables
    v = prog.NewContinuousVariables(N, "v")
    w = prog.NewContinuousVariables(N, "w")
    # t = np.linspace(0, tf, n_timesteps+1)

    # # Add a constraint that sets the final time to be greater than the initial time
    # t0 = 0.0
    # prog.AddLinearConstraint(tf >= t0)

    # Add the initial and final position and orientation constraints
    prog.AddLinearConstraint(x[0] == x_initial)
    prog.AddLinearConstraint(y[0] == y_initial)
    prog.AddLinearConstraint(theta[0] == theta_initial)
    prog.AddLinearConstraint(x[-1] == x_final)
    prog.AddLinearConstraint(y[-1] == y_final)
    prog.AddLinearConstraint(theta[-1] == theta_final)

    # Add the dynamic constraints
    for i in range(0, N - 1):
        prog.AddConstraint(x[i + 1] == x[i] + v[i] * cos(theta[i]) * h)
        prog.AddConstraint(y[i + 1] == y[i] + v[i] * sin(theta[i]) * h)
        prog.AddConstraint(theta[i + 1] == theta[i] + w[i] * h)
        prog.AddCost(h)

    # Add the control constraints
    for i in range(0, N):
        prog.AddConstraint(v[i] >= 0)
        prog.AddConstraint(v[i] <= max_v)
        prog.AddConstraint(w[i] >= -max_w)
        prog.AddConstraint(w[i] <= max_w)

    # # Add the velocity and steering angle limits
    # prog.AddBoundingBoxConstraint(-max_v, max_v, v[i])
    # prog.AddBoundingBoxConstraint(-max_w, max_w, w[i])

    # Add the dynamic constraints
    # for i in range(n_timesteps):

    #     ti = i * dt
    #     k1 = ackerman.f(np.hstack((x[i], v[i])), ti)
    #     k2 = ackerman.f(np.hstack((x[i], v[i])) + dt / 2 * k1, ti + dt / 2)
    #     k3 = ackerman.f(np.hstack((x[i], v[i])) + dt / 2 * k2, ti + dt / 2)
    #     k4 = ackerman.f(np.hstack((x[i], v[i])) + dt * k3, ti + dt)
    #     x_next = x[i] + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
    #     prog.AddConstraint(
    #         x_next == rk4_step(ackerman.f, [x[i], y[i], theta[i]], [v[i], w[i]], dt)
    #     )

    # x_next = prog.NewContinuousVariables(3, "x_next" + str(i))
    # prog.AddConstraint(
    #     x_next=rk4_step(ackerman.f, [x[i], y[i], theta[i]], [v[i], w[i]], dt)
    # )
    # prog.AddConstraint(x[i + 1] == x_next[0])
    # prog.AddConstraint(y[i + 1] == x_next[1])
    # prog.AddConstraint(theta[i + 1] == x_next[2])

    # State derivatives using Forward Euler
    # prog.AddConstraint(x[i + 1] == x[i] + v[i] * cos(theta[i]) * dt)
    # prog.AddConstraint(y[i + 1] == y[i] + v[i] * sin(theta[i]) * dt)
    # prog.AddConstraint(theta[i + 1] == theta[i] + w * dt)
    # prog.AddConstraint(v[i + 1] == v[i] + v_steer[i] * dt)

    # # Add the velocity and steering angle limits
    # prog.AddBoundingBoxConstraint(-max_v, max_v, v[i])
    # prog.AddBoundingBoxConstraint(-max_w, max_w, w[i])

    # prog.AddConstraint(-max_v <= v[i])
    # prog.AddConstraint(v[i] <= max_v)
    # prog.AddConstraint(-max_v_steer <= v_steer[i])
    # prog.AddConstraint(v_steer[i] <= max_v_steer)
    # prog.AddConstraint(-delta_max <= delta[i])
    # prog.AddConstraint(delta[i] <= delta_max)

    # Define the cost function
    # total_time = prog.NewContinuousVariables(1, "total_time")[0]
    # prog.AddCost(total_time)

    # Add the time constraints
    # prog.AddConstraint(t >= 0)
    # prog.AddConstraint(t <= 10)

    # # Define energy consumption
    # wheel_torque = prog.NewContinuousVariables(N - 1, "wheel_torque")
    # wheel_angular_velocity = prog.NewContinuousVariables(
    #     N - 1, "wheel_angular_velocity"
    # )
    # wheel_power = wheel_torque * wheel_angular_velocity
    # total_power = np.sum(np.abs(wheel_power)) * h

    # # Define accuracy as a constraint
    # position_tolerance = 0.01
    # prog.AddConstraint(position_error <= position_tolerance)

    # # Define energy consumption as a constraint
    # energy_tolerance = 100.0
    # prog.AddConstraint(total_power <= energy_tolerance)

    # # Add the straight line initial guess to the decision variables
    # x_guess = np.linspace(x_initial, x_final, N + 1)
    # y_guess = np.linspace(y_initial, y_final, N + 1)
    # theta_guess = np.linspace(theta_initial, theta_final, N + 1)
    # v_guess = np.full(N, (0.001 + max_v) / 2)
    # w_guess = np.full(N, (0.001 + max_w) / 2)

    # print("x: ", len(x))
    # print("x_guess: ", len(x_guess))
    # print("y: ", len(y))
    # print("y_guess: ", len(y_guess))
    # print("theta: ", len(theta))
    # print("theta_guess: ", len(theta_guess))
    # print("v: ", len(v))
    # print("v_guess: ", len(v_guess))
    # print("delta: ", len(w))
    # print("delta_guess: ", len(w_guess))
    # prog.SetInitialGuess(x, x_guess)
    # prog.SetInitialGuess(y, y_guess)
    # prog.SetInitialGuess(theta, theta_guess)
    # prog.SetInitialGuess(v, v_guess)
    # prog.SetInitialGuess(w, w_guess)

    # Add the initial guess
    for i in range(N + 1):
        prog.SetInitialGuess(x[i], (i * x_final + (N - i) * x_initial) / N)
        prog.SetInitialGuess(y[i], (i * y_final + (N - i) * y_initial) / N)
        prog.SetInitialGuess(theta[i], (i * theta_initial + (N - i) * 0) / N)

    for i in range(N):
        prog.SetInitialGuess(v[i], max_v / 2 * (1 - np.cos(i * np.pi / N)))
        prog.SetInitialGuess(w[i], 0)

    # Solve the optimization problem
    result = Solve(prog)
    assert result.is_success()

    # Get the optimal trajectory
    x_opt = result.GetSolution(x)
    y_opt = result.GetSolution(y)
    theta_opt = result.GetSolution(theta)

    # print("v_opt: ", v_opt)
    # print("w_opt: ", w_opt)

    # # Plot the optimal trajectory
    # plt.plot(x_opt, y_opt)
    # plt.xlabel("X position")
    # plt.ylabel("Y position")
    # plt.show()

    return x_opt, y_opt, theta_opt
