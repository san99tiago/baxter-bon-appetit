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
    b1 = bc.baxterClass()
    right_limb = baxter_interface.Limb('right')
    left_limb = baxter_interface.Limb('left')
    left_limb_names = left_limb.joint_names()
    right_limb_names = right_limb.joint_names()

    if limb == 'right':
        joints_values = b1.ipk(tm_w0_tool, 'r', 'u')
        joint_command = convert_list2dict(right_limb_names, joints_values)
        print(joint_command)
        right_limb.move_to_joint_positions(joint_command)

    if limb == 'left':
        joints_values = b1.ipk(tm_w0_tool, 'l', 'u')
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
        tm_w0_tool = np.array(
            [[-0.11347392,  0.48550109, -0.86684045, -1.01236464],
             [-0.03542536, -0.87389812, -0.48481659, -0.54508842],
             [-0.99290922, -0.0243059,   0.1163637,   1.12681583],
             [0,          0,          0,          1.]]
        )
        limb = "right"

        move_baxter_based_on_transformation_matrix(tm_w0_tool, limb)

    except rospy.ROSInterruptException:
        print('---------- baxter error ------------')
