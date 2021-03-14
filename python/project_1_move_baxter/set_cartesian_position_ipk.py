#!/usr/bin/env python
# Import the main modules
import rospy
import BaxterClass as bc
import baxter_interface

import numpy as np

def set_position(fpk,arm):
    Baxter = bc.baxterClass()
    right_arm = baxter_interface.Limb('right')
    left_arm = baxter_interface.Limb('left')
    lj = left_arm.joint_names()
    rj = right_arm.joint_names()
    print(rj)

    if arm == 'right':
        joints_values = Baxter.ipk(fpk,'r','u')
        joint_command = convert_list2dict(rj, joints_values)
        print(joint_command)
        right_arm.move_to_joint_positions(joint_command)

    if arm == 'left':
        joints_values = Baxter.ipk(fpk,'l','u')
        joint_command = convert_list2dict(lj, joints_values)
        print(joint_command)
        left_arm.move_to_joint_positions(joint_command)

def convert_list2dict(keys,values):
    j_command = {
        keys[0]:values[0],
        keys[1]:values[1],
        keys[2]:values[2],
        keys[3]:values[3],
        keys[4]:values[4],
        keys[5]:values[5],
        keys[6]:values[6]
    }
    return j_command


def main():
    rospy.init_node("set_cartesian_position_ipk")
    fpk = np.array(
[[-0.11347392,  0.48550109, -0.86684045, -1.01236464],
 [-0.03542536, -0.87389812, -0.48481659, -0.54508842],
 [-0.99290922, -0.0243059,   0.1163637,   1.12681583],
 [ 0,          0,          0,          1.        ]]
)
    set_position(fpk,'right')

#     fpk = np.array(
# [[ 0.0033672,   0.99373637, -0.11169913,  0.16828152],
#  [-0.194894,   -0.10890567, -0.9747594,  -1.21174888],
#  [-0.98081853,  0.0250517,   0.19330655,  1.27058042],
#  [ 0,          0,          0,          1.        ]]
#         )
#     set_position(fpk,'left')


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
		print 'BAXTER ERROR'