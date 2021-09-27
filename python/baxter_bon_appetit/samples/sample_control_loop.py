#!/usr/bin/python
#-*- coding: utf-8 -*
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


def calculate_cartesian_vectors(current_thetas):
    # CURRENT CARTESIAN POSITION CALCULATIONS...
    tm_current = bc.BaxterClass().fpk(current_thetas, "right", 7)
    current_position = tm_current[0:3, 3]
    current_orientation = transf.Transformation(
        0, 0, 0, [0, 0, 0]).get_fixed_angles_from_tm(tm_current)

    return np.concatenate([current_position, current_orientation], axis=0).reshape(6, 1)


def test_1_step_response_without_feedback(show_results=True):
    """
    Sample loop to plot step response with constant change in each DOF without 
    any control algorithm (just to see Baxter's "chaos" response)
    """

    # Variables for simulation
    x_k = np.matrix([[0.1], [0.15], [0.2], [0.25], [0.3], [0.35], [0.4]])
    u_k = np.matrix([[0.01], [0.01], [0.01], [0.01], [0.01], [0.01], [0.01]])

    # Desired cartesian goal [x_g, y_g, z_g, x_angle_g, y_angle_g, z_angle_g]
    # (NOT any goal, just to be able to plot)
    cartesian_goal = np.array([0, 0, 0, 0, 0, 0]).reshape(6, 1)

    iteration_vector = list()
    x_matrix = np.zeros((x_k.shape[0], 0))
    u_matrix = np.zeros((u_k.shape[0], 0))
    cartesian_matrix = np.zeros((cartesian_goal.shape[0], 0))
    cartesian_goal_matrix = np.zeros((cartesian_goal.shape[0], 0))

    total_time_in_seconds = 5
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
        print("iteration_vector:")
        print(iteration_vector)
        print("len(iteration_vector):")
        print(len(iteration_vector))
        print("u_matrix:")
        print(u_matrix)
        print("x_matrix:")
        print(x_matrix)
        print("x_matrix.shape:")
        print(x_matrix.shape)

    create_plots(
        iteration_vector,
        cartesian_matrix,
        cartesian_goal_matrix,
        sample_time_in_seconds,
        "Cartesian Values responses based on step respone no feedback",
        "current",
        "fake-goal"
    )

    create_plots(
        iteration_vector,
        x_matrix,
        u_matrix,
        sample_time_in_seconds,
        "X and U vectors response based on step respone no feedback",
        "x",
        "u"
    )

    plt.show()


def test_2_mpc_first_attempt(show_results=True):
    """
    Sample control loop to test MPC algorithm on Baxter right limbr for custom 
    variables such as N, total_time, sample_time, cartesian_goal, x0, u0 and 
    validate the resulting plots with or without noise.
    """

    # Main conditions for executing the control loop with MPC algorithm
    N = 1  # Prediction horizon
    total_time_in_seconds = 20
    sample_time_in_seconds = 0.1

    # Initial conditions for states and inputs
    x0 = np.array(
        [
            0.39500005288049406,
            -1.2831749290661485,
            -0.18867963690990588,
            2.5905100555414924,
            -0.11428156869746332,
            -1.3506700837331067,
            0.11504855909140603
        ]
    ).reshape(7, 1)

    u0 = np.array([0, 0, 0, 0, 0, 0, 0]).reshape(7, 1)

    # Number inputs (same as number of degrees of freedom)
    nu = u0.shape[0]

    # Initial cartesian_goal "default" value
    cartesian_goal = np.array(
        [
            [
                -0.9,
                -1.0,
                1.1,
                0.6660425877100662,
                1.5192944057794895,
                -1.3616725381467032
            ],
        ] * N
    ).transpose().reshape(6, N)

    # ---------- Main Control loop -------------
    # Variables for control loop
    x_k = x0
    u_k = u0

    iteration_vector = list()
    x_matrix = np.zeros((x_k.shape[0], 0))
    u_matrix = np.zeros((u_k.shape[0], 0))
    cartesian_matrix = np.zeros((cartesian_goal.shape[0], 0))
    cartesian_goal_matrix = np.zeros((cartesian_goal.shape[0], 0))

    # Instead of running the algorithms in real time, we will run the total
    # amount of discrete iterations (to get the total time)...
    iteration = 0
    total_iterations = int(total_time_in_seconds/sample_time_in_seconds)
    for _ in range(total_iterations):
        iteration_vector.append(iteration)
        if (show_results == True):
            print("Iteration (k): ", iteration)
        iteration = iteration + 1

        # Apply MPC prediction
        mpc = b_mpc.MpcController(N, True, True)
        cartesian_goal = cartesian_goal + np.array(
            [
                [
                    0.001 * np.sin(iteration/5),
                    0.001 * np.sin(iteration/5),
                    0.001 * np.sin(iteration/5),
                    0,
                    0,
                    0
                ],
            ] * N
        ).transpose().reshape((6, N))

        dict_results = mpc.execute_mpc(cartesian_goal, x_k)
        u = dict_results["optimal_dthetas"]

        # Prediction Horizon for 1 iteration at a time
        u_k = u[:, 0].reshape((nu, 1))

        # Calculate new states based on StateSpace representation
        x_k_plus_1 = x_k + u_k

        # Add "random noise" to measurements (like real-life)
        x_k_plus_1 = x_k_plus_1 + np.array(
            [
                1 * random.uniform(-0.005, 0.005),
                1 * random.uniform(-0.005, 0.005),
                1 * random.uniform(-0.005, 0.005),
                1 * random.uniform(-0.005, 0.005),
                1 * random.uniform(-0.005, 0.005),
                1 * random.uniform(-0.005, 0.005),
                1 * random.uniform(-0.005, 0.005)
            ]
        ).reshape((7, 1))

        # Update current state for the next iteration
        x_k = x_k_plus_1

        cartesian_k = calculate_cartesian_vectors(x_k)

        # Save current x and u values to plot latter
        x_matrix = np.hstack((x_matrix, x_k))
        u_matrix = np.hstack((u_matrix, u_k))
        cartesian_matrix = np.hstack((cartesian_matrix, cartesian_k))
        cartesian_goal_matrix = np.hstack(
            (cartesian_goal_matrix, cartesian_goal[:, 0].reshape(6, 1)))

    if (show_results == True):
        print("len(iteration_vector):")
        print(len(iteration_vector))
        print("iteration_vector:")
        print(iteration_vector)
        print("u_matrix.shape:")
        print(u_matrix.shape)
        print("u_matrix:")
        print(u_matrix)
        print("x_matrix.shape:")
        print(x_matrix.shape)
        print("x_matrix:")
        print(x_matrix)

    create_plots(
        iteration_vector,
        cartesian_matrix,
        cartesian_goal_matrix,
        sample_time_in_seconds,
        "Cartesian Values responses based on MPC with N={}".format(N),
        "current",
        "goal"
    )

    create_plots(
        iteration_vector,
        x_matrix,
        u_matrix,
        sample_time_in_seconds,
        "X and U responses based on MPC with N={}".format(N),
        "x",
        "u"
    )

    plt.show()

def test_3_mpc_with_control_horizon(show_results=True):
    """
    Sample control loop to test MPC algorithm on Baxter right limbr for custom 
    variables such as N, M, total_time, sample_time, cartesian_goal, x0, u0 and 
    validate the resulting plots with or without noise.
    """

    # Main conditions for executing the control loop with MPC algorithm
    N = 1  # Prediction horizon
    M = 1  # Control horizon
    m_count = 1  # Control horizon counter (1, 2, ... , M, 1, 2, ..., M, 1, 2, ..., M ...)

    total_time_in_seconds = 10
    sample_time_in_seconds = 0.1

    # Initial conditions for states and inputs
    x0 = np.array(
        [
            0.39500005288049406,
            -1.2831749290661485,
            -0.18867963690990588,
            2.5905100555414924,
            -0.11428156869746332,
            -1.3506700837331067,
            0.11504855909140603
        ]
    ).reshape(7, 1)

    u0 = np.array([0, 0, 0, 0, 0, 0, 0]).reshape(7, 1)

    # Number inputs (same as number of degrees of freedom)
    nu = u0.shape[0]

    # Initial cartesian_goal "default" value
    cartesian_goal = np.array(
        [
            [
                -0.9,
                -1.0,
                1.1,
                0.6660425877100662,
                1.5192944057794895,
                -1.3616725381467032
            ],
        ] * N
    ).transpose().reshape(6, N)

    # ---------- Main Control loop -------------
    # Variables for control loop
    x_k = x0
    u_k = u0

    iteration_vector = list()
    x_matrix = np.zeros((x_k.shape[0], 0))
    u_matrix = np.zeros((u_k.shape[0], 0))
    cartesian_matrix = np.zeros((cartesian_goal.shape[0], 0))
    cartesian_goal_matrix = np.zeros((cartesian_goal.shape[0], 0))

    # Instead of running the algorithms in real time, we will run the total
    # amount of discrete iterations (to get the total time)...
    iteration = 0
    total_iterations = int(total_time_in_seconds/sample_time_in_seconds)
    for _ in range(total_iterations):
        iteration_vector.append(iteration)
        if (show_results == True):
            print("Iteration (k): ", iteration)
        iteration = iteration + 1

        # Apply MPC prediction
        if (m_count >= M):
            m_count = 1

        if (m_count == 1):
            mpc = b_mpc.MpcController(N, True, True)
            cartesian_goal = cartesian_goal + np.array(
                [
                    [
                        0 * np.sin(iteration/5),
                        0 * np.sin(iteration/5),
                        0 * np.sin(iteration/5),
                        0,
                        0,
                        0
                    ],
                ] * N
            ).transpose().reshape((6, N))

            dict_results = mpc.execute_mpc(cartesian_goal, x_k)
            u = dict_results["optimal_dthetas"]

        # Prediction Horizon for 1 iteration at a time
        u_k = u[:, m_count - 1].reshape((nu, 1))

        # Modify counter to pass to next control horizon value
        m_count = m_count + 1

        # Calculate new states based on StateSpace representation
        x_k_plus_1 = x_k + u_k

        # Add "random noise" to measurements (like real-life)
        x_k_plus_1 = x_k_plus_1 + np.array(
            [
                1 * random.uniform(-0.005, 0.005),
                1 * random.uniform(-0.005, 0.005),
                1 * random.uniform(-0.005, 0.005),
                1 * random.uniform(-0.005, 0.005),
                1 * random.uniform(-0.005, 0.005),
                1 * random.uniform(-0.005, 0.005),
                1 * random.uniform(-0.005, 0.005)
            ]
        ).reshape((7, 1))

        # Update current state for the next iteration
        x_k = x_k_plus_1

        cartesian_k = calculate_cartesian_vectors(x_k)

        # Save current x and u values to plot latter
        x_matrix = np.hstack((x_matrix, x_k))
        u_matrix = np.hstack((u_matrix, u_k))
        cartesian_matrix = np.hstack((cartesian_matrix, cartesian_k))
        cartesian_goal_matrix = np.hstack(
            (cartesian_goal_matrix, cartesian_goal[:, 0].reshape(6, 1)))

    if (show_results == True):
        print("len(iteration_vector):")
        print(len(iteration_vector))
        print("iteration_vector:")
        print(iteration_vector)
        print("u_matrix.shape:")
        print(u_matrix.shape)
        print("u_matrix:")
        print(u_matrix)
        print("x_matrix.shape:")
        print(x_matrix.shape)
        print("x_matrix:")
        print(x_matrix)

    create_plots(
        iteration_vector,
        cartesian_matrix,
        cartesian_goal_matrix,
        sample_time_in_seconds,
        "Cartesian Values responses based on MPC with N={}".format(N),
        "current",
        "goal"
    )

    create_plots(
        iteration_vector,
        x_matrix,
        u_matrix,
        sample_time_in_seconds,
        "X and U responses based on MPC with N={}".format(N),
        "x",
        "u"
    )

    plt.show()


if __name__ == '__main__':
    # test_1_step_response_without_feedback(True)
    # test_2_mpc_first_attempt(True)
    test_3_mpc_with_control_horizon(True)
