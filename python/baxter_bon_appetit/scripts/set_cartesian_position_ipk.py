#!/usr/bin/env python

# Own imports
import baxter_essentials.baxter_class as bc

# General module imports
import numpy as np
import rospy
import baxter_interface


def move_baxter_based_on_transformation_matrix(tm_w0_tool, limb):
    """
    Move Baxter's desired limb based on a complete transformation matrix.
    """
    rospy.init_node("set_cartesian_position_ipk")
    set_position(tm_w0_tool, limb)


def set_position(tm_w0_tool, limb):
    """
    Generate Baxter's commands to move arms based on Baxter Interface.
    """
    b1 = bc.BaxterClass()
    right_limb = baxter_interface.Limb('right')
    left_limb = baxter_interface.Limb('left')
    left_limb_names = left_limb.joint_names()
    right_limb_names = right_limb.joint_names()

    if limb == 'right':
        joints_values = b1.ipk(tm_w0_tool, 'right', 'up')
        joint_command = convert_list2dict(right_limb_names, joints_values)
        print(joint_command)
        right_limb.move_to_joint_positions(joint_command)

    if limb == 'left':
        joints_values = b1.ipk(tm_w0_tool, 'left', 'up')
        joint_command = convert_list2dict(left_limb_names, joints_values)
        print(joint_command)
        left_limb.move_to_joint_positions(joint_command)


def convert_list2dict(keys, values):
    """
    Convert general key-values list to specific Baxter's dictionary for joints.
    """
    j_command = {
        keys[0]: values[0],
        keys[1]: values[1],
        keys[2]: values[2],
        keys[3]: values[3],
        keys[4]: values[4],
        keys[5]: values[5],
        keys[6]: values[6]
    }
    return j_command


if __name__ == '__main__':
    try:
        # tm_w0_tool = np.array(

        #     [[-0.04854143, 0.99879321, -0.00747311, -0.69522343],
        #     [-0.15331122, -0.01484379, -0.98806646, -0.96854655],
        #     [-0.986985, -0.04681645, 0.15384675, 1.23731447],
        #     [ 0, 0, 0, 1]]

        # )
        # limb = "right"
        # move_baxter_based_on_transformation_matrix(tm_w0_tool, limb)

        tm_w0_tool = bc.BaxterClass().TM_left_limb_camera
        limb = "left"
        move_baxter_based_on_transformation_matrix(tm_w0_tool, limb)

        tm_w0_tool = bc.BaxterClass().TM_right_limb_home
        limb = "right"
        move_baxter_based_on_transformation_matrix(tm_w0_tool, limb)


    except rospy.ROSInterruptException:
        print('---------- baxter error ------------')
