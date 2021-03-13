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
    # fpk = np.array([[-0.04594029,  0.96221089, -0.2684021,  -0.46561621],
    #     [ 0.13083041, -0.26057597, -0.95654773, -0.88335946],
    #     [-0.9903397, -0.07905924, -0.11391551,  1.00567349],
    #     [ 0,          0,          0,          1.        ]])
    # set_position(fpk,'right')

    fpk = np.array([[ 0.9564598,  -0.15358337, -0.24818702,  0.17581124],
        [-0.23760143,  0.08411014, -0.96771434, -0.94997619],
        [ 0.16949988,  0.98454946,  0.04395634,  0.90462043],
        [ 0,          0,          0,          1        ]])
    set_position(fpk,'left')


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
		print 'BAXTER ERROR'