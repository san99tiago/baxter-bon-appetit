#!/usr/bin/env python

# Built-in imports
import sys

# Own imports
import baxter_essentials.baxter_class as bc

# General module imports
import numpy as np
import rospy
from sensor_msgs.msg import JointState


def joint_states_callback(event):
    """
    Callback to create a global variable to save current joint_states angles.
    """
    global joint_states
    baxter_angles = event.position
    joint_states = {
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


def map_baxter_workspace(limb, save_info, point_or_matrix):
    """
    Map Baxter's Workspace based on the desired Input/Output.
    """
    global workspace_file

    # Initialize main ROS node for mapping the workspace.
    rospy.init_node('baxter_mapping', anonymous=True)
    rospy.Subscriber('/robot/joint_states', JointState, joint_states_callback)
    rate = rospy.Rate(50)

    # Create Baxter's instance to use forward-pose-kinematics method
    b1 = bc.BaxterClass()

    while not rospy.is_shutdown():

        # Specify the type of data to save based on desired output
        if (point_or_matrix == "point"):
            # Only save the specific X-Y-Z coordinates
            data = b1.fpk(joint_states[limb], limb, 7)[:3, 3:4]

            # Create readable output structure for X-Y-Z coordinates
            output = '[' + str(data[0, 0])+',' + \
                str(data[1, 0])+','+str(data[2, 0])+']\n'

        elif (point_or_matrix == "matrix"):
            # Save the complete transformation matrix
            print(joint_states[limb])
            data = b1.fpk(joint_states[limb], limb, 7)
            output = np.array2string(data)

        if (save_info == "y"):
            workspace_file.write(output)

        print(output)
        rate.sleep()


if __name__ == '__main__':
    try:
        # Get terminal input arguments
        limb = sys.argv[1]  # "left" or "right"
        save_info = sys.argv[2]  # "y" or "n"
        point_or_matrix = sys.argv[3]  # "point" or "matrix"

        # Initialize global variables
        joint_states = {'right': [0, 0, 0, 0, 0,
                                  0, 0], 'left': [0, 0, 0, 0, 0, 0, 0]}

        # Verify if we save the info or not
        if (save_info == "y"):
            # Create/Open the main file to save the Workspace information
            workspace_file = open(sys.argv[4], 'a')
            map_baxter_workspace(limb, save_info, point_or_matrix)
            workspace_file.close()
        else:
            map_baxter_workspace(limb, save_info, point_or_matrix)

    except rospy.ROSInterruptException:
        print('---------- baxter error ------------')
