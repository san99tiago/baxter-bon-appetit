#!/usr/bin/env python

# Built-int imports
import sys
import time

# Own imports
import baxter_essentials.baxter_class as bc
import baxter_essentials.transformation as transf
import baxter_control.mpc_controller as b_mpc
import baxter_control.cartesian_increment as c_inc

# General module imports
import numpy as np
import rospy
import baxter_interface

from geometry_msgs.msg import (
    Pose
)

from sensor_msgs.msg import (
    JointState
)

from baxter_core_msgs.msg import (
    JointCommand
)

from std_msgs.msg import (
    String
)


class MpcControl:
    """
    ROS Node that subscribes to the face_coordinates publisher and enables the 
    MPC controller to determine the best possible values for the next degrees
    of freedom of each Baxter's joint.

    :param sample_time: integer that defines the sample_time in seconds.
    """

    def __init__(self, sample_time, prediction_horizon=1):

        # Prediction horizon
        self.N = prediction_horizon

        # Control sample time in seconds
        self.sample_time = sample_time

        # Initial conditions for states and inputs
        self.x0 = np.array(
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

        # Initial conditions for inputs (delta joint values)
        self.u0 = np.array([0, 0, 0, 0, 0, 0, 0]).reshape(7, 1)

        # Initial cartesian_goal "default" value
        self.cartesian_goal = np.array(
            [
                [
                    0,
                    0,
                    0,
                    0.6660425877100662,
                    1.5192944057794895,
                    -1.3616725381467032
                ],
            ] * self.N
        ).transpose().reshape(6, self.N)

        _joint_states_sub = rospy.Subscriber(
            '/robot/joint_states',
            JointState,
            self.joint_states_callback,
            queue_size=1
        )

        _fsm_sub = rospy.Subscriber(
            'user/fsm',
            String,
            self.update_fsm_callback,
            queue_size=1
        )
        self.state = "stop"

        _face_coordinates_sub = rospy.Subscriber(
            'user/face_coordinates',
            Pose,
            self.update_coordinates_callback,
            queue_size=1
        )

        self._pub_joint_control_values = rospy.Publisher(
            'user/joint_control_values',
            JointCommand,
            queue_size=1
        )

    def joint_states_callback(self, event):
        """
        Callback to get current joint_states angles for Baxter robot.
        """
        baxter_angles = event.position
        self.joint_states = {
            'right': [
                baxter_angles[11],
                baxter_angles[12],
                baxter_angles[9],
                baxter_angles[10],
                baxter_angles[13],
                baxter_angles[14],
                baxter_angles[15]
            ],
            'left': [
                baxter_angles[4],
                baxter_angles[5],
                baxter_angles[2],
                baxter_angles[3],
                baxter_angles[6],
                baxter_angles[7],
                baxter_angles[8]
            ]
        }

    def update_fsm_callback(self, std_string):
        """
        Recieve the callback function from the current node that publishes the 
        fsm as a "String" std_msgs. This enables the node to keep updating the 
        Finite State Machine values for executing the "mpc_control".
        :param geometry_pose: current fsm message with a standard 
            "String" format from "std_msgs.msg". 
        """
        self.state = std_string.data
        print(self.state)

    def update_coordinates_callback(self, geometry_pose):
        """
        Recieve the callback function from the currect node that publishes the 
        face_coordinates as a "Pose" geometry_message. This callback calls the 
        necessary methods to apply the MPC algorithms for Baxter's DOF.

        :param geometry_pose: current face_coordinates message with a standard 
            "Pose" format from "geometry_msgs.msg". 
        """
        # Desired cartesian goal [x_g, y_g, z_g, x_angle_g, y_angle_g, z_angle_g]
        self.cartesian_goal = np.array(
            [
                [
                    geometry_pose.position.x,
                    geometry_pose.position.y,
                    geometry_pose.position.z,
                    0.6660425877100662,
                    1.5192944057794895,
                    -1.3616725381467032
                ],
            ] * self.N
        ).transpose().reshape(6, self.N)

        # Get current Baxter right limb cartesian positions
        b1 = bc.BaxterClass()
        cartesian_current = b1.fpk(
            self.joint_states["right"], "right", 7)[:3, 3:4]

        # Add extra zeros (for cartesian orientation)
        open_loop_cartesian_goal = self.cartesian_goal
        cartesian_current = np.concatenate(
            [cartesian_current, np.array([0, 0, 0]).reshape(3, 1)])

        c1 = c_inc.CartesianIncrements(
            open_loop_cartesian_goal, cartesian_current)
        self.current_position_vector = cartesian_current + \
            c1.calculate_cartesian_increment()

        # We calculate the complete TM matrix (for proportional control if the
        # ... MPC optimal solution fails):
        self.tm_w0_tool = self.create_tm_structure_from_pose_and_rotation()
        print(self.tm_w0_tool)

    def get_joint_values_from_baxter_transformation_matrix(self):
        """
        Get Baxter's right limb joint values based on a complete transformation 
        matrix (in order to publish).
        """
        # Get current joint values from Baxter right limb
        b1 = bc.BaxterClass()
        return b1.ipk(self.tm_w0_tool, 'right', 'up')

    def create_tm_structure_from_pose_and_rotation(self):
        """
        Create a Homogeneous Transformation Matrix from a rotation matrix and 
        a position vector.
        :returns: transformation matrix numpy array of size (4x4).
        """
        # Create a matrix (3x4) from rotation matrix and position vector
        tm_top_part = np.concatenate(
            [self.ROTATION_MATRIX, self.current_position_vector[:3]], 1
        )

        # Add the lower part array to the transformation matrix
        lower_part_array = np.array([[0, 0, 0, 1]])

        return np.concatenate([tm_top_part, lower_part_array], 0)

    def execute_mpc_control(self, show_results=True):
        """
        Main control loop to execute MPC controller based on the TM of the
        user's face_coordinates and it publishes the predicted joint_values
        in the "user/joint_control_values" topic when the MPC prediction
        is optimal (otherwise, it keeps the last calculated value).
        """
        # Number of states and inputs
        nu = self.u0.shape[0]

        # ---------- Main Control loop -------------
        # Variables for control loop
        x_k = self.x0
        u_k = self.u0

        last_time = 0
        iteration = 0

        while not rospy.is_shutdown():
            # Only proceed to control calculation in correct sample time multiple
            sample_time_condition = time.time() - last_time >= self.sample_time
            # Only proceed to control if the face was detected
            face_detected_condition = self.cartesian_goal[2] != 0
            # Only proceed to control if the FSM is in "mpc" state
            state_mpc_condition = "mpc" == self.state

            if (sample_time_condition and face_detected_condition and state_mpc_condition):
                # Update time conditions and iterations
                last_time = time.time()
                if (show_results == True):
                    print("Iteration (k): ", iteration)
                iteration = iteration + 1

                # Apply MPC prediction
                try:
                    mpc = b_mpc.MpcController(self.N, True, True)

                    # Read current Baxter right limb joint values
                    x_k = self.joint_states["right"]
                    x_k = np.array(x_k).reshape(7, 1)

                    dict_results = mpc.execute_mpc(self.cartesian_goal, x_k)
                    u = dict_results["optimal_dthetas"]

                    # Prediction Horizon for 1 iteration at a time
                    u_k = u[:, 0].reshape((nu, 1))

                    # Calculate new states based on StateSpace representation
                    self.x_k_plus_1 = x_k + u_k

                except Exception as e:
                    print("***** There was no optimal solution *****")
                    print("***** Applying Proportional Control *****")

                # Publish "/user/joint_control_values" topic
                self.publish_control_joint_commands()

                # ! Temporary control action to test MPC implementation ...
                # ! will be replaced by control node
                self.move_baxter_based_on_joint_values()

            else:
                if (face_detected_condition == False):
                    print("Face NOT detected")

    def publish_control_joint_commands(self):
        """
        Publish 'JointCommand' topic with the desired joint-values for each of
        Baxter's right limb based on the MPC predictions.
        """
        cmd = JointCommand()
        cmd.mode = JointCommand.POSITION_MODE
        cmd.names = [
            "right_s0",
            "right_s1",
            "right_e0",
            "right_e1",
            "right_w0",
            "right_w1",
            "right_w2"
        ]
        cmd.command = self.x_k_plus_1
        self._pub_joint_control_values.publish(cmd)

    def move_baxter_based_on_joint_values(self):
        """
        Move Baxter's right limb based on calculated joint_values from the MPC.
        """
        # Create the baxter_inteface instance to work with Baxter's right limb
        self.right_limb = baxter_interface.Limb('right')
        right_limb_names = self.right_limb.joint_names()

        joint_command = dict(zip(right_limb_names, self.x_k_plus_1))
        print(joint_command)
        self.right_limb.set_joint_positions(joint_command)


def main():
    print("Initializing node... ")
    rospy.init_node('mpc_control')

    # Prediction horizon (input)
    N = sys.argv[1]

    main_node_mpc = MpcControl(0.001, N)
    main_node_mpc.execute_mpc_control()
    return 0


if __name__ == '__main__':
    sys.exit(main())
