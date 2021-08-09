#!/usr/bin/env python

# Built-in imports
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

from geometry_msgs.msg import (
    Pose
)

from sensor_msgs.msg import (
    JointState
)

from baxter_core_msgs.msg import (
    JointCommand
)


class NodeProportionalControlFromFaceCoordinates:
    """
    ROS Node that subscribes to the face_coordinates publisher and enables the 
    baxter_interface control method to apply a proportional-control action 
    command to move the Baxter's right limb to specific position-orientation.

    :param rospy_rate: integer that defines the frequency for the ROS nodes.
    """

    def __init__(self, rospy_rate):
        self.define_rotation_matrix()
        self.rate = rospy.Rate(rospy_rate)

        # Initial right limb matrix as "default" value
        self.tm_w0_tool = bc.BaxterClass().TM_right_limb_home

        # Initial current_position_vector as "default" value
        self.current_position_vector = np.array([0, 0, 0]).reshape((3, 1))

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

        _joint_states_sub = rospy.Subscriber(
            '/robot/joint_states',
            JointState,
            self.joint_states_callback,
            queue_size=1
        )

        self._pub_joint_control_values = rospy.Publisher(
            'user/joint_control_values',
            JointCommand,
            queue_size=1
        )

    def update_fsm_callback(self, std_string):
        """
        Recieve the callback function from the current node that publishes the 
        fsm as a "String" std_msgs. This enables the node to keep updating the 
        Finite State Machine values for executing the "open_loop_control".
        :param geometry_pose: current fsm message with a standard 
            "String" format from "std_msgs.msg". 
        """
        self.state = std_string.data
        print(self.state)

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

    def define_rotation_matrix(self):
        """
        This method defines a constant attribute for the right limb correct 
        orientation in the feeding process. This matrix was found by empiric 
        experiments with Baxter.
        """
        self.ROTATION_MATRIX = np.array(
            [
                [-0.04483493,  0.99897278, -0.00657433],
                [-0.15247979, -0.01334699, -0.98821646],
                [-0.98728909, -0.04330416,  0.15292157]
            ]
        )

    def update_coordinates_callback(self, geometry_pose):
        """
        Recieve the callback function from the current node that publishes the 
        face_coordinates as a "Pose" geometry_message. This callback calls the 
        necessary methods to apply the proportional-control algorithms based 
        on BaxterInterface class.
        :param geometry_pose: current face_coordinates message with a standard 
            "Pose" format from "geometry_msgs.msg". 
        """
        self.current_position_vector = np.array(
            [
                geometry_pose.position.x,
                geometry_pose.position.y,
                geometry_pose.position.z
            ]
        ).reshape((3, 1))
        self.tm_w0_tool = self.create_tm_structure_from_pose_and_rotation()
        print(self.tm_w0_tool)

    def create_tm_structure_from_pose_and_rotation(self):
        """
        Create a Homogeneous Transformation Matrix from a rotation matrix and 
        a position vector.
        :returns: transformation matrix numpy array of size (4x4).
        """
        # Create a matrix (3x4) from rotation matrix and position vector
        tm_top_part = np.concatenate(
            [self.ROTATION_MATRIX, self.current_position_vector], 1
        )

        # Add the lower part array to the transformation matrix
        lower_part_array = np.array([[0, 0, 0, 1]])

        return np.concatenate([tm_top_part, lower_part_array], 0)

    def update_control_joint_values(self):
        """
        Move Baxter's right limb based on a complete transformation matrix 
        using BaxterInterface class with a proportional control.
        """
        # Get current joint values from Baxter right limb
        b1 = bc.BaxterClass()
        self.control_joints_values = b1.ipk(self.tm_w0_tool, 'right', 'up')

    def execute_control(self):
        """
        Execute main control loop for Baxter's right arm.
        """
        while not rospy.is_shutdown():
            if (self.state == "open_loop"):
                if (self.current_position_vector[0] != 0):
                    print("Face detected")
                    self.update_control_joint_values()
                    self.publish_control_joint_commands()
                    # self.rate.sleep()
                else:
                    print("Face NOT detected")
            else:
                self.publish_current_joint_angles()

    def publish_control_joint_commands(self):
        """
        Publish 'JointCommand' topic with the desired joint-values for each of
        Baxter's right limb based on the open loop control.
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
        cmd.command = self.control_joints_values
        self._pub_joint_control_values.publish(cmd)

    def publish_current_joint_angles(self):
        """
        Publish 'JointCommand' topic with the current joint-values for each of
        Baxter's right limb based on the open loop control.
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
        cmd.command = self.joint_states["right"]
        self._pub_joint_control_values.publish(cmd)


def main():
    print("Initializing node... ")
    rospy.init_node('open_loop_control')

    main_node_proportional_control = NodeProportionalControlFromFaceCoordinates(
        100)

    main_node_proportional_control.execute_control()

    return 0


if __name__ == '__main__':
    sys.exit(main())
