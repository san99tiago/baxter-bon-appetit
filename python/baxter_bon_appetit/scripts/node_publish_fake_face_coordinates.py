#!/usr/bin/env python

# Built-int imports
import sys
import numpy as np

# General module imports
import rospy

from geometry_msgs.msg import (
    Pose
)


class NodePublishFaceCoordinates:
    """
    ROS Node that publishes FAKE current face coordinates of the patient, in 
    order to test correctly other Baxter functionalities in Simulations.
    :param camera_resolution: tuple with camera resolution. Example: (1280,800).
    :param rospy_rate: integer that defines the frequency for the ROS nodes.
    """

    def __init__(self, camera_resolution, rospy_rate):
        self.baxter_camera_name = "left_hand_camera"
        self.camera_resolution = camera_resolution
        self.rate = rospy.Rate(rospy_rate)

        print("THIS METHOD IS A <FAKE> METHOD TO TEST OTHER NODES WITH THIS TOPIC.")

        self._pub_face_coordinates = rospy.Publisher(
            'user/face_coordinates',
            Pose,
            queue_size=1)

    def execute_publish_coordinates(self):
        """
        Execute the FAKE publisher for the face coordinates in an infinite loop 
        based on the rospy_rate given in the __init__ method.
        """

        self.current_coordinates = np.array(
            [
                float(sys.argv[1]),
                float(sys.argv[2]),
                float(sys.argv[3])
            ]
        ).reshape((3, 1))

        while not rospy.is_shutdown():
            # Acquire coordinates that are being updated from the method
            # ... "limb_cam_process_image_callback"
            coordinates = self.current_coordinates

            # Create Pose message based on "geometry_msgs"
            p = Pose()
            p.position.x = coordinates[0]
            p.position.y = coordinates[1]
            p.position.z = coordinates[2]

            # Make sure the quaternion is valid and normalized
            p.orientation.x = 0.0
            p.orientation.y = 0.0
            p.orientation.z = 0.0
            p.orientation.w = 1.0

            # Publish node and apply frequency conditions
            self._pub_face_coordinates.publish(p)
            self.rate.sleep()


def main():
    print("Initializing node... ")
    rospy.init_node('publish_face_coordinates', anonymous=True)

    main_node_face_coordinates = NodePublishFaceCoordinates((640, 400), 100)
    main_node_face_coordinates.execute_publish_coordinates()

    return 0


if __name__ == '__main__':
    sys.exit(main())
