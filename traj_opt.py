import numpy as np
from pydrake.all import MathematicalProgram, Solve, DirectCollocation, sqrt
from pydrake.systems.meshcat_visualizer import MeshcatVisualizer
from matplotlib import pyplot as plt
import math

def solve_traj_opt(x_initial, y_initial, theta_initial, x_final, y_final, theta_final):

    # Create the optimization problem
    prog = MathematicalProgram()

    # Initial conditions for intial position and goal position
    # x_initial = 2
    # y_initial = 2
    # theta_initial = 0
    # x_final = 8
    # y_final = 8
    # theta_final = np.pi/2
    robot_radius = 0.4
    v_max = 1.0
    w_max = 0.5

    # Add decision variables for the robot's position and velocity
    n_timesteps = 50
    dt = 0.0001
    # states
    x = prog.NewContinuousVariables(n_timesteps + 1, "x")
    y = prog.NewContinuousVariables(n_timesteps + 1, "y")
    theta = prog.NewContinuousVariables(n_timesteps + 1, "theta")
    # inputs
    v = prog.NewContinuousVariables(n_timesteps, "v")
    w = prog.NewContinuousVariables(n_timesteps, "w")

    # Add a cost that minimizes the final time tf
    tf = prog.NewContinuousVariables(1, "tf")[0]
    prog.AddCost(tf)

    # Add a constraint that sets the final time to be greater than the initial time
    t0 = 0.0
    prog.AddLinearConstraint(tf >= t0)

    # Add the initial and final position and orientation constraints
    prog.AddLinearConstraint(x[0] == x_initial)
    prog.AddLinearConstraint(y[0] == y_initial)
    prog.AddLinearConstraint(theta[0] == theta_initial)
    prog.AddLinearConstraint(x[-1] == x_final)
    prog.AddLinearConstraint(y[-1] == y_final)
    prog.AddLinearConstraint(theta[-1] == theta_final)

    # Add the dynamics constraints for each timestep
    for k in range(n_timesteps - 1):
        # x_k = np.array([x[k], y[k], theta[k]])
        # x_dot_k = np.array([v[k] * np.cos(theta[k]), v[k] * np.sin(theta[k]), w[k]])
        # x_kp1 = np.array([x[k+1], y[k+1], theta[k+1]])
        # x_dot_kp1 = np.array([v[k+1] * np.cos(theta[k+1]), v[k+1] * np.sin(theta[k+1]), w[k+1]])
        # h_k = dt
        # f_k = x_dot_k - (x_kp1 - x_k) / h_k
        # prog.AddConstraint(f_k[0] == 0)
        # prog.AddConstraint(f_k[1] == 0)
        # prog.AddConstraint(f_k[2] == 0)
        # # Add the bounds on the control inputs
        # prog.AddConstraint(v[k] >= 0)
        # prog.AddConstraint(w[k] >= -np.pi / 4)
        # prog.AddConstraint(w[k] <= np.pi / 4)
        x_next = x[k] + dt * (v[k] * np.cos(theta[k]) + w[k] * np.sin(theta[k]))
        y_next = y[k] + dt * (v[k] * np.sin(theta[k]) - w[k] * np.cos(theta[k]))
        theta_next = theta[k] + dt * w[k]
        prog.AddConstraint(x[k+1] == x_next)
        prog.AddConstraint(y[k+1] == y_next)
        prog.AddConstraint(theta[k+1] == theta_next)
        prog.AddConstraint(v[k+1] <= v_max)
        prog.AddConstraint(w[k+1] <= w_max)

    # Add the cost function
    # cost = sum(v**2 + w**2)
    # prog.AddQuadraticCost(cost)

    # # Add a cost that minimizes the total energy consumption
    # P = 1.0  # power consumption rate
    # k_energy = 1.0
    # energy = sum([P * (v[i]**2 + w[i]**2) * dt for i in range(n_timesteps)])
    # prog.AddCost(k_energy * energy)

    # # Add a cost that maximizes the accuracy of the final position and orientation
    # k_position = 1.0
    # k_orientation = 1.0
    # position_error = sqrt((x[-1]-x_final)**2 + (y[-1]-y_final)**2)
    # orientation_error = abs(theta[-1]-theta_final)
    # accuracy = -k_position * position_error - k_orientation * orientation_error
    # prog.AddCost(accuracy)

    # Add a cost that penalizes large changes in velocity and angular velocity
    # v_smooth = (v[1:] - v[:-1])**2
    # w_smooth = (w[1:] - w[:-1])**2
    # k_smooth = 1
    # prog.AddCost(k_smooth * (sum(v_smooth) + sum(w_smooth)))

    # Add the initial guess for the trajectory as a straight line from the initial to final position
    x_guess = np.linspace(x_initial, x_final, n_timesteps+1)
    y_guess = np.linspace(y_initial, y_final, n_timesteps+1)
    theta_guess = np.linspace(theta_initial, theta_final, n_timesteps+1)
    v_guess = np.full(n_timesteps, (0.001+v_max)/2)
    w_guess = np.full(n_timesteps, (0.001+w_max)/2)

    prog.SetInitialGuess(x, x_guess)
    prog.SetInitialGuess(y, y_guess)
    prog.SetInitialGuess(theta, theta_guess)
    prog.SetInitialGuess(v, v_guess)
    prog.SetInitialGuess(w, w_guess)
    # Set the initial guess for the time and input trajectories
    prog.SetInitialGuess(tf, 2.0)

    # Solve the optimization problem
    result = Solve(prog)
    assert result.is_success()

    # Get the optimal trajectory
    x_opt = result.GetSolution(x)
    y_opt = result.GetSolution(y)
    theta_opt = result.GetSolution(theta)
    v_opt = result.GetSolution(v)
    w_opt = result.GetSolution(w)

    # print("v_opt: ", v_opt)
    # print("w_opt: ", w_opt)

    # # Plot the optimal trajectory
    # plt.plot(x_opt, y_opt)
    # plt.xlabel("X position")
    # plt.ylabel("Y position")
    # plt.show()

    return x_opt, y_opt, theta_opt
