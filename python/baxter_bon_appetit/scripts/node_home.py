#!/usr/bin/env python

# Built-int imports
import sys

# Own imports
import baxter_essentials.baxter_class as bc

# General module imports
import numpy as np
import rospy
import baxter_interface

class NodeGoToHome:
    def __init__(self):

        # TODO: create FEM topic on another node and subscribe to it from here

        self.go_to_home()

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
        limb.move_to_joint_positions(joint_command)

    def go_to_home(self):
        # Move left limb to the home position
        tm_w0_tool = bc.BaxterClass().TM_left_limb_camera
        limb = "left"
        self.move_baxter_based_on_transformation_matrix(tm_w0_tool, limb)

        # Move right limb to the home position
        tm_w0_tool = bc.BaxterClass().TM_right_limb_home
        limb = "right"
        self.move_baxter_based_on_transformation_matrix(tm_w0_tool, limb)


def main():
    print("Initializing node... ")
    rospy.init_node('go_to_home')
    main_node_go_to_home = NodeGoToHome()
    return 0


if __name__ == '__main__':
    sys.exit(main())
