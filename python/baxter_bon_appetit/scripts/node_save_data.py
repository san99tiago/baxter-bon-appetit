#!/usr/bin/env python

# Built-int imports
import sys
import time
import datetime

# Own imports
import baxter_essentials.baxter_class as bc
import baxter_essentials.transformation as transf

# General module imports
import numpy as np
import matplotlib.pyplot as plt
import rospy

from sensor_msgs.msg import (
    JointState
)

from geometry_msgs.msg import (
    Pose
)

from baxter_core_msgs.msg import (
    JointCommand
)


class NodeSaveData:
    """
    ROS Node that enables Baxter to save data based on desired data-types. The
    goal of this node is to save important experimental values in files for 
    future analysis of each control strategy.
    """

    def __init__(self, sample_time, total_time, title, file_name):

        # Sample time (Ts) in seconds
        self.sample_time = sample_time

        # Total execution time in seconds
        self.total_time = total_time

        # Title for each plot (customized for each control strategy)
        self.title = title

        # Desired filename for the output after processing the data
        self.file_name = file_name

        self.show_results = True
        self.save_data = True

        _joint_states_sub = rospy.Subscriber(
            '/robot/joint_states',
            JointState,
            self.update_joint_states_callback,
            queue_size=1
        )

        _face_coordinates_sub = rospy.Subscriber(
            'user/face_coordinates',
            Pose,
            self.update_face_coordinates_callback,
            queue_size=1
        )

        _joint_control_values_sub = rospy.Subscriber(
            'user/joint_control_values',
            JointCommand,
            self.control_joint_values_callback,
            queue_size=1
        )

    def update_joint_states_callback(self, joint_state):
        """
        Callback to get current joint_states angles for Baxter robot.
        :param joint_state: current joint values message with a "sensor_msgs" 
            format from "sensor_msgs.msg". 
        """
        baxter_angles = joint_state.position
        self.current_joint_states = np.array(
            [
                baxter_angles[11],
                baxter_angles[12],
                baxter_angles[9],
                baxter_angles[10],
                baxter_angles[13],
                baxter_angles[14],
                baxter_angles[15]
            ]).reshape(7, 1)

        if (self.show_results == True):
            print("joint_states_callback: ")
            print(self.current_joint_states)

    def update_face_coordinates_callback(self, geometry_pose):
        """
        Recieve the callback function from the current node that publishes the 
        face_coordinates as a "Pose" geometry_message. 
        :param geometry_pose: current face_coordinates message with a standard 
            "Pose" format from "geometry_msgs.msg". 
        """
        self.current_cartesian_goal = np.array(
            [
                geometry_pose.position.x,
                geometry_pose.position.y,
                geometry_pose.position.z,
                1.3486466410078362,
                1.466247306243215,
                -1.2373949601695735
            ]).reshape((6, 1))

        if (self.show_results == True):
            print("face_coordinates_callback: ")
            print(self.current_cartesian_goal)

    def control_joint_values_callback(self, joint_command):
        """
        Recieve the callback function from the current node that publishes the 
        joint_control_values (control effort) as a "JointCommand" 
        baxter_core_msgs.
        :param joint_command: current control_joint  message with a standard 
            "JointCommand" format from "baxter_core_msgs.msg". 
        """
        self.current_control_joint_names = joint_command.names
        self.current_control_joint_values = np.array(
            [joint_command.command]).reshape(7, 1)

        if (self.show_results == True):
            print("control_joint_values_callback: ")
            print(self.current_control_joint_names)
            print(self.current_control_joint_values)

    def execute_data_analysis(self):
        """
        Loop parameterized by the sample_time and the total_time to acquire the 
        necessary signals for plotting the results of Baxter control algorithms.
        """
        # Main conditions for correct sample times and loop execution
        final_time = time.time() + self.total_time
        last_time = 0
        iteration = 0

        # Prepare initial conditions for first values (before callbacks)
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
        cartesian_values_0 = np.array([0, 0, 0, 0, 0, 0]).reshape(6, 1)
        cartesian_goal_0 = np.array([0, 0, 0, 0, 0, 0]).reshape(6, 1)

        # Initialize values for the initial execution of the loop
        self.current_joint_states = x0
        self.current_control_joint_values = u0
        self.current_cartesian_values = cartesian_values_0
        self.current_cartesian_goal = cartesian_goal_0

        # Main variables to plot the output correctly
        self.iteration_vector = list()
        self.x_matrix = np.zeros((x0.shape[0], 0))
        self.u_matrix = np.zeros((u0.shape[0], 0))
        self.cartesian_matrix = np.zeros((cartesian_goal_0.shape[0], 0))
        self.cartesian_goal_matrix = np.zeros((cartesian_goal_0.shape[0], 0))

        while (time.time() < final_time):
            # Only proceed to data-analysis in correct sample time multiples
            if (time.time() - last_time >= self.sample_time):
                # Update time conditions and iterations
                last_time = time.time()
                self.iteration_vector.append(iteration)
                if (self.show_results == True):
                    print("Iteration (k): ", iteration)
                iteration = iteration + 1

                # Calculate current cartesian vector for "self.cartesian_matrix"
                self.calculate_cartesian_vectors()

                # Save current matrix values to plot latter correctly
                self.x_matrix = np.hstack(
                    (self.x_matrix, self.current_joint_states)
                )

                self.u_matrix = np.hstack(
                    (self.u_matrix, self.current_control_joint_values)
                )

                self.cartesian_matrix = np.hstack(
                    (self.cartesian_matrix, self.current_cartesian_values)
                )

                self.cartesian_goal_matrix = np.hstack(
                    (self.cartesian_goal_matrix, self.current_cartesian_goal)
                )

                # Save data in txt file
                if (self.save_data == True):
                    self.save_current_iteration_data()

        # After executing the complete loop time (after total_time)...
        if (self.show_results == True):
            print("iteration_vector: ")
            print(self.iteration_vector)
            print(len(self.iteration_vector))
            print("x_matrix: ")
            print(self.x_matrix)
            print(self.x_matrix.shape)
            print("u_matrix: ")
            print(self.u_matrix)
            print(self.u_matrix.shape)
            print("cartesian_matrix: ")
            print(self.cartesian_matrix)
            print(self.cartesian_matrix.shape)
            print("cartesian_goal_matrix: ")
            print(self.cartesian_goal_matrix)
            print(self.cartesian_goal_matrix.shape)

        # Execute main methods to plot all data-outputs
        self.create_plots(
            self.iteration_vector,
            self.cartesian_matrix,
            self.cartesian_goal_matrix,
            self.sample_time,
            "Cartesian Values responses based on {}".format(self.title),
            "current",
            "goal"
        )

        self.create_plots(
            self.iteration_vector,
            self.x_matrix,
            self.u_matrix,
            self.sample_time,
            "X and U vectors response based on {}".format(self.title),
            "x",
            "u"
        )

        plt.show()

    def calculate_cartesian_vectors(self):
        """
        Current cartesian position and orientation calculations from the 
        current joint_states that are recieved from the callback.
        """
        tm_current = bc.BaxterClass().fpk(
            self.current_joint_states,
            "right",
            7
        )
        current_position = tm_current[0:3, 3]
        current_orientation = transf.Transformation(
            0, 0, 0, [0, 0, 0]).get_fixed_angles_from_tm(tm_current)

        self.current_cartesian_values = np.concatenate(
            [current_position, current_orientation], axis=0
        ).reshape(6, 1)

    def save_current_iteration_data(self):
        """
        Main method to save current iterations data for the joint_values, the 
        control_joint_efforts, the cartesian_current and the cartesian_goal.
        """
        # Create/Open the main file to save the Workspace information
        current_date = datetime.datetime.now().strftime("%Y_%m_%d__%H_%M_")
        file_name = current_date + self.file_name
        workspace_file = open(file_name, 'a')

        # Create readable output structure for the data:
        # ["right_s0", "right_s1", "right_e0", "right_e1", "right_w0", ...
        #  "right_w1", "right_w2", "eff_right_s0", "eff_right_s1", ...
        #  "eff_right_e0", "eff_right_e1", "eff_right_w0", "eff_right_w1", ...
        #  "eff_right_w2", "x_current", "y_current", "z_current", ...
        #  "x_a_current", "y_a_current", z_a_current", "x_goal", "y_goal", ...
        #  "z_goal", "x_a_goal", "y_a_goal", "z_a_goal"]
        output = '[' + \
            str(self.current_joint_states[0]) + ',' + \
            str(self.current_joint_states[1]) + ',' + \
            str(self.current_joint_states[2]) + ',' + \
            str(self.current_joint_states[3]) + ',' + \
            str(self.current_joint_states[4]) + ',' + \
            str(self.current_joint_states[5]) + ',' + \
            str(self.current_joint_states[6]) + ',' + \
            str(self.current_control_joint_values[0]) + ',' + \
            str(self.current_control_joint_values[1]) + ',' + \
            str(self.current_control_joint_values[2]) + ',' + \
            str(self.current_control_joint_values[3]) + ',' + \
            str(self.current_control_joint_values[4]) + ',' + \
            str(self.current_control_joint_values[5]) + ',' + \
            str(self.current_control_joint_values[6]) + ',' + \
            str(self.current_cartesian_values[0]) + ',' + \
            str(self.current_cartesian_values[1]) + ',' + \
            str(self.current_cartesian_values[2]) + ',' + \
            str(self.current_cartesian_values[3]) + ',' + \
            str(self.current_cartesian_values[4]) + ',' + \
            str(self.current_cartesian_values[5]) + ',' + \
            str(self.current_cartesian_goal[0]) + ',' + \
            str(self.current_cartesian_goal[1]) + ',' + \
            str(self.current_cartesian_goal[2]) + ',' + \
            str(self.current_cartesian_goal[3]) + ',' + \
            str(self.current_cartesian_goal[4]) + ',' + \
            str(self.current_cartesian_goal[5]) + ',' + ']\n'

        workspace_file.write(output)
        workspace_file.close()

    def create_plots(self, iteration_vector, x_matrix, u_matrix, sample_time, title, name1, name2):
        """
        Create plots for automatically showing the joint values of each DOF and 
        the cartesian variables for Baxter control algorithms.
        :param iteration_vector: list of each iteration of the control loop.
        :param x_matrix: numpy array of dimensions (7, NK) with each value 
            for the joint_states of Baxter robot.
        :param u_matrix: numpy array of dimensions (7, NK) with each value 
            for the control_joint_efforts of Baxter robot.
        :param sample_time: time between each data-capture (Ts in seconds).
        :param title: name of the control strategy for the plots (String).
        :param name1: name of the labels for each plot (legends).
            Example "x" or "current".
        :param name2: name of the labels for each plot (legends).
            Example "u" or "goal".
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


def main():
    print("Initializing node... ")
    rospy.init_node('save_data')

    # Acquire inputs for the params of the NodeSaveData class
    if (len(sys.argv) < 5):
        print("Enter the arguments: sample_time, total_time, title, file_name")
        return 0
    sample_time = float(sys.argv[1])
    total_time = float(sys.argv[2])
    title = sys.argv[3]
    file_name = sys.argv[4]

    main_node_save_data = NodeSaveData(
        sample_time, total_time, title, file_name)

    main_node_save_data.execute_data_analysis()


if __name__ == '__main__':
    sys.exit(main())
