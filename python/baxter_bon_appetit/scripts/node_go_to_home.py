#!/usr/bin/env python

# Built-int imports
import sys

# Own imports
import baxter_essentials.baxter_class as bc

# General module imports
import numpy as np
import rospy
import baxter_interface

from std_msgs.msg import (
    String
)


class NodeGoToHome:
    """
    ROS Node that enables the startup of Baxter robot, so that it can move to
    the initial "home" position for both of its limbs.
    """

    def __init__(self):
        _fsm_sub = rospy.Subscriber(
            'user/fsm',
            String,
            self.update_fsm_callback,
            queue_size=1
        )
        self.state = "stop"
        self.go_to_home()

    def update_fsm_callback(self, std_string):
        """
        Recieve the callback function from the current node that publishes the 
        fsm as a "String" std_msgs. This enables the node to keep updating the 
        Finite State Machine values for executing the "go_to_home" movements.
        :param geometry_pose: current fsm message with a standard 
            "String" format from "std_msgs.msg". 
        """
        self.state = std_string.data
        print(self.state)

    def move_baxter_based_on_transformation_matrix(self, tm_w0_tool, limb_to_move):
        """
        Move Baxter's limb based on a complete transformation matrix using
        BaxterInterface class.
        :param tm_w0_tool: Transformation Matrix from W0
            (origin of the workspace), to Baxter's Tool (end of the arm).
        :param limb: selected Baxter arm.
            example: "left", "right".
        """

        b1 = bc.BaxterClass()
        limb = baxter_interface.Limb(limb_to_move)
        limb_names = limb.joint_names()
        joints_values = b1.ipk(tm_w0_tool, limb_to_move, 'up')
        joint_command = dict(zip(limb_names, joints_values))
        print(joint_command)
        limb.set_joint_positions(joint_command)

    def move_baxter_based_on_joint_values(self, joint_values, limb_to_move):
        """
        Move Baxter's limb based on a each DOF joint values using
        BaxterInterface class.
        :param joint_values: Joint Values for desired Baxter position.
        :param limb: selected Baxter arm.
            example: "left", "right".
        """

        limb = baxter_interface.Limb(limb_to_move)
        limb_names = limb.joint_names()
        joint_command = dict(zip(limb_names, joint_values))
        print(joint_command)
        limb.set_joint_positions(joint_command)

    def go_to_home(self):
        """
        Method to move each limb to the desired home position based on the 
        current state given by the FSM (self.state) value.
        """
        while not rospy.is_shutdown():
            if (self.state == "go_to_home"):
                print("Executing <go_to_home> movements")
                # Move left limb to the home position based on TM
                tm_w0_tool = bc.BaxterClass().TM_left_limb_camera
                limb = "left"
                self.move_baxter_based_on_transformation_matrix(tm_w0_tool, limb)

                # Move right limb to the home position based on joint_values
                joint_values = bc.BaxterClass().joint_values_right_limb_home
                limb = "right"
                self.move_baxter_based_on_transformation_matrix(joint_values, limb)

                # Alway set the state to "stop", after executing the movements
                self.state = "stop"


def main():
    print("Initializing node... ")
    rospy.init_node('go_to_home')
    main_node_go_to_home = NodeGoToHome()
    return 0


if __name__ == '__main__':
    sys.exit(main())
