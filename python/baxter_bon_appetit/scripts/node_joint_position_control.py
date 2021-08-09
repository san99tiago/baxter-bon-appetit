#!/usr/bin/env python

# Built-in imports
import time
import sys

# General module imports
import rospy
import baxter_interface

from baxter_core_msgs.msg import (
    JointCommand
)

from std_msgs.msg import (
    String
)

class NodeJointPositionControl:
    """
    ROS Node that enables Baxter Joint Controllers to execute the commands to 
    move Baxter's right limb to a desired position.

    :param sample_time: integer that defines the sample_time in seconds.
    """

    def __init__(self, sample_time):

        # Control sample time in seconds
        self.sample_time = sample_time

        # Initialize command values for first iteration
        self.command_names = [
            "right_s0",
            "right_s1",
            "right_e0",
            "right_e1",
            "right_w0",
            "right_w1",
            "right_w2"
        ]
        self.command_values = [0, 0, 0, 0, 0, 0, 0]

        _fsm_sub = rospy.Subscriber(
            'user/fsm',
            String,
            self.update_fsm_callback,
            queue_size=1
        )
        self.state = "stop"

        _joint_control_values_sub = rospy.Subscriber(
            'user/joint_control_values',
            JointCommand,
            self.joint_control_values_callback,
            queue_size=1
        )

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

    def joint_control_values_callback(self, joint_command):
        """
        Callback to get current control_joint_values for Baxter robot.
        """
        self.command_names = joint_command.names
        self.command_values = joint_command.command

    def execute_control(self):
        """
        Execute main control loop for Baxter's right limb.
        """
        last_time = 0
        iteration = 0

        while not rospy.is_shutdown():
            # Only proceed to control calculation in correct sample time multiple
            sample_time_condition = time.time() - last_time >= self.sample_time
            # Only proceed to control if the control joint values are right
            correct_values_condition = self.command_values[0] != 0
            # Only proceed to control if FSM is in "mpc" or "open_loop" state
            correct_state_condition = self.state == "mpc" or self.state == "open_loop"

            if (sample_time_condition and correct_values_condition and correct_state_condition):
                # Update time conditions, iterations and execute control method
                last_time = time.time()
                iteration = iteration + 1
                self.move_baxter_based_on_joint_values()

    def move_baxter_based_on_joint_values(self):
        """
        Move Baxter's right limb based on control joint values from callback.
        """
        # Create dict structure for control command (from command callback)
        joint_command = dict(zip(self.command_names, self.command_values))
        print(joint_command)

        # Create the baxter_inteface instance to work with Baxter's right limb
        self.right_limb = baxter_interface.Limb('right')
        self.right_limb.set_joint_positions(joint_command)


def main():
    print("Initializing node... ")
    rospy.init_node('joint_position_control')

    main_node_joint_position_control = NodeJointPositionControl(0.001)
    main_node_joint_position_control.execute_control()

    return 0


if __name__ == '__main__':
    sys.exit(main())
