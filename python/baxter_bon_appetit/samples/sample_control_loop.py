#! /usr/bin/python
# -*- coding: utf-8 -*
# SAMPLE FOR SIMPLE CONTROL LOOP TO IMPLEMENT BAXTER_CONTROL MPC ALGORITHMS


"""
MPC sample tracking for Baxter's right limb with specific references.
Authors: Santiago Garcia Arango and Elkin Javier Guerra Galeano.
"""

# Built-int imports
import time
import random

# Own imports
import baxter_essentials.baxter_class as bc
import baxter_essentials.transformation as transf
import baxter_control.mpc_controller as b_mpc


# General module imports
import numpy as np
import matplotlib.pyplot as plt


def create_plots(iteration_vector, x_matrix, u_matrix, sample_time, title, name1, name2):
    """
    Create simple simulation plots based on vectors
    It returns two pop-out matplotlib graphs.
    """

    # Define a figure for the creation of the plot
    figure_1, all_axes = plt.subplots(x_matrix.shape[0], 1)

    current_axes = 0
    for axes_i in all_axes:
        # Generate the plot and its axes for each Xi and Ui.
        axes_i.plot(iteration_vector,
                    x_matrix[current_axes, :].T, 'b', linewidth=1)
        axes_i.plot(iteration_vector,
                    u_matrix[current_axes, :].T, 'g', linewidth=1)
        current_axes = current_axes + 1

        # Customize figure with the specific "x"-"y" labels
        if (current_axes <= 3):
            if (name1 == "x"):
                axes_i.set_ylabel("[rad]")
            else:
                axes_i.set_ylabel("[m]")
        else:
            axes_i.set_ylabel("[rad]")

        # Add labels to each subplot
        axes_i.legend(["{}{}".format(name1, current_axes),
                       "{}{}".format(name2, current_axes)])

        # Remove inner axes layers (only leave the outer ones)
        axes_i.label_outer()

        # Add personalized text to each subplot (at lower-right side)
        axes_i.text(0.98,
                    0.02,
                    'SGA-EJGG',
                    verticalalignment='bottom',
                    horizontalalignment='right',
                    transform=axes_i.transAxes,
                    color='black',
                    fontsize=6
                    )

        # Add grid
        axes_i.grid(color='black', linestyle='-', alpha=0.2, linewidth=1)

    # Change the background color of the external part
    figure_1.patch.set_facecolor((0.2, 1, 1))

    # Configure plot title and horizontal x-label
    all_axes[0].set_title(title)
    all_axes[len(
        all_axes) - 1].set_xlabel("Iterations [k] (Ts={} seconds)".format(sample_time))


def show_plots():
    # Display all the created figures of this script
    plt.show()


def calculate_cartesian_vectors(current_thetas):
    # CURRENT CARTESIAN POSITION CALCULATIONS...
    tm_current = bc.BaxterClass().fpk(current_thetas, "right", 7)
    current_position = tm_current[0:3, 3]
    current_orientation = transf.Transformation(
        0, 0, 0, [0, 0, 0]).get_fixed_angles_from_tm(tm_current)

    return np.concatenate([current_position, current_orientation], axis=0).reshape(6, 1)


def test_1_open_loop(show_results):
    # ---------- Main Control loop -------------
    # Variables for control loop
    x_k = np.matrix([[0.1], [0.2], [0.3], [0.4], [0.5], [0.6], [0.7]])
    u_k = np.matrix([[0.01], [0.01], [0.01], [0.01], [0.01], [0.01], [0.01]])

    # Desired cartesian goal [x_g, y_g, z_g, x_angle_g, y_angle_g, z_angle_g]
    # (NOT any goal, just to be able to plot)
    cartesian_goal = np.array([0, 0, 0, 0, 0, 0]).reshape(6, 1)

    iteration_vector = list()
    x_matrix = np.zeros((x_k.shape[0], 0))
    u_matrix = np.zeros((u_k.shape[0], 0))
    cartesian_matrix = np.zeros((cartesian_goal.shape[0], 0))
    cartesian_goal_matrix = np.zeros((cartesian_goal.shape[0], 0))

    total_time_in_seconds = 10
    sample_time_in_seconds = 0.01
    final_time = time.time() + total_time_in_seconds
    last_time = 0
    iteration = 0

    while (time.time() < final_time):
        if (time.time() - last_time >= sample_time_in_seconds):
            last_time = time.time()
            iteration_vector.append(iteration)
            if (show_results == True):
                print("Iteration (k): ", iteration)
            iteration = iteration + 1

            x_k_plus_1 = x_k + u_k
            x_k = x_k_plus_1

            cartesian_k = calculate_cartesian_vectors(x_k)

            x_matrix = np.hstack((x_matrix, x_k_plus_1))
            u_matrix = np.hstack((u_matrix, u_k))
            cartesian_matrix = np.hstack((cartesian_matrix, cartesian_k))
            cartesian_goal_matrix = np.hstack(
                (cartesian_goal_matrix, cartesian_goal))

    if (show_results == True):
        print(iteration_vector)
        print(len(iteration_vector))
        print(x_matrix)
        print(x_matrix.shape)

    create_plots(iteration_vector, cartesian_matrix, cartesian_goal_matrix, sample_time_in_seconds,
                 "Cartesian Values responses based on Open Loop", "current", "goal")

    create_plots(iteration_vector, x_matrix, u_matrix, sample_time_in_seconds,
                 "X and U vectors response based on Open Loop", "x", "u")

    show_plots()


def test_2_mpc_first_attempt(show_results=True):
    N = 1  # Prediction horizon

    # Initial conditions for states and inputs
    x0 = np.array([0.8, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]).reshape(7, 1)
    u0 = np.array([0, 0, 0, 0, 0, 0, 0]).reshape(7, 1)

    # Number of states and inputs
    nx = x0.shape[0]
    nu = u0.shape[0]

    # Desired cartesian goal [x_g, y_g, z_g, x_angle_g, y_angle_g, z_angle_g]
    cartesian_goal = np.array(
        [-0.5, -0.8, 1.0, 2.5, -0.3, -0.2] * N).reshape(6, N)

    # ---------- Main Control loop -------------
    # Variables for control loop
    x_k = x0
    u_k = u0

    iteration_vector = list()
    x_matrix = np.zeros((x_k.shape[0], 0))
    u_matrix = np.zeros((u_k.shape[0], 0))
    cartesian_matrix = np.zeros((cartesian_goal.shape[0], 0))
    cartesian_goal_matrix = np.zeros((cartesian_goal.shape[0], 0))

    total_time_in_seconds = 10
    sample_time_in_seconds = 0.01
    final_time = time.time() + total_time_in_seconds
    last_time = 0
    iteration = 0

    while (time.time() < final_time):
        # Only proceed to control calculation in correct sample time multiple
        if (time.time() - last_time >= sample_time_in_seconds):
            # Update time conditions and iterations
            last_time = time.time()
            iteration_vector.append(iteration)
            if (show_results == True):
                print("Iteration (k): ", iteration)
            iteration = iteration + 1

            # Apply MPC prediction
            mpc = b_mpc.MpcController(N, True, True)
            cartesian_goal = cartesian_goal + np.array(
                [0.005 * np.sin(iteration/5),
                 0.005 * np.sin(iteration/5),
                 0.005 * np.sin(iteration/5),
                 0,
                 0,
                 0]
            ).reshape((6, N))
            dict_results = mpc.execute_mpc(cartesian_goal, x_k)
            u = dict_results["optimal_dthetas"]

            # Prediction Horizon for 1 iteration at a time
            u_k = u[:, 0].reshape((nu, 1))

            # Calculate new states based on StateSpace representation
            x_k_plus_1 = x_k + u_k

            # Add "random noise" to measurements (like real-life)
            x_k_plus_1 = x_k_plus_1 + np.array(
                [random.uniform(-0.01, 0.01),
                 random.uniform(-0.01, 0.01),
                 random.uniform(-0.01, 0.01),
                 random.uniform(-0.01, 0.01),
                 random.uniform(-0.01, 0.01),
                 random.uniform(-0.01, 0.01),
                 random.uniform(-0.01, 0.01)]
            ).reshape((7, N))

            # Update current state for the next iteration
            x_k = x_k_plus_1

            cartesian_k = calculate_cartesian_vectors(x_k)

            # Save current x and u values to plot latter
            x_matrix = np.hstack((x_matrix, x_k))
            u_matrix = np.hstack((u_matrix, u_k))
            cartesian_matrix = np.hstack((cartesian_matrix, cartesian_k))
            cartesian_goal_matrix = np.hstack(
                (cartesian_goal_matrix, cartesian_goal))

    if (show_results == True):
        print("len(iteration_vector): ", len(iteration_vector))
        print("iteration_vector:", iteration_vector)
        print("u_matrix.shape: ", u_matrix.shape)
        print("u_matrix:", u_matrix)
        print("x_matrix.shape: ", x_matrix.shape)
        print("x_matrix:", x_matrix)

    create_plots(iteration_vector, cartesian_matrix, cartesian_goal_matrix, sample_time_in_seconds,
                 "Cartesian Values responses based on MPC", "current", "goal")

    create_plots(iteration_vector, x_matrix, u_matrix, sample_time_in_seconds,
                 "X and U responses based on MPC", "x", "u")

    show_plots()


if __name__ == '__main__':
    DEBUG_ = True
    # test_1_open_loop(True)
    test_2_mpc_first_attempt(False)
