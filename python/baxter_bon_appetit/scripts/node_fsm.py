#!/usr/bin/env python

# Built-in imports
import sys

# General module imports
import rospy

from std_msgs.msg import (
    String
)


class NodeFiniteStateMachine:
    """
    ROS Node that publishes the current state of the Finite State Machine for 
    the general functionalities of Baxter Bon Appetit algorithms.
    :param rospy_rate: integer that defines the frequency for the ROS nodes.
    """

    def __init__(self, rospy_rate):
        self.rate = rospy.Rate(rospy_rate)

        self._pub_face_coordinates = rospy.Publisher(
            'user/fsm',
            String,
            queue_size=1
        )

    def publish_fsm_state(self, state):
        """
        Method to publish the desired state in a ROS topic that enables the 
        overall selection of each one of the other nodes.
        :param state: flag that defines the desired state.
            For example, "go_to_home", "get_food", "mpc", "open_loop", "stop".
        """
        while not rospy.is_shutdown():
            self._pub_face_coordinates.publish(state)


def main():
    print("Initializing node... ")
    rospy.init_node('publish_fsm')
    main_node_publish_fsm = NodeFiniteStateMachine(100)
    # Publish the user's desired state from the system's arg param
    main_node_publish_fsm.publish_fsm_state(sys.argv[1])
    return 0


if __name__ == '__main__':
    sys.exit(main())
