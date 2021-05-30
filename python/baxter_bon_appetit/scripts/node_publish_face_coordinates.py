#!/usr/bin/env python

# Built-int imports
import time
import sys
import numpy as np

# Own imports
import face_detect_hc as fdhc
import baxter_vision_mapping.baxter_camera_complete_transform as bcp

# General module imports
import rospy
import cv2 as cv
import cv_bridge

from baxter_interface.camera import (
    CameraController,
)
from sensor_msgs.msg import (
    Image,
)

from geometry_msgs.msg import (
    Pose
)


class NodePublishFaceCoordinates:
    """
    ROS Node that publishes the current face coordinates of the patient, based 
    on the face-detection Computer Vision algorithms and the default Baxter 
    node for the left_hand_camera.
    :param camera_resolution: tuple with camera resolution. Example: (1280,800).
    :param rospy_rate: integer that defines the frequency for the ROS nodes.
    """

    def __init__(self, camera_resolution, rospy_rate):
        self.baxter_camera_name = "left_hand_camera"
        self.camera_resolution = camera_resolution
        self.rate = rospy.Rate(rospy_rate)

        self.validate_camera_resolution()
        self.close_all_cameras()
        self.open_camera()

        _image_sub = rospy.Subscriber(
            '/cameras/{}/image'.format(self.baxter_camera_name),
            Image,
            self.limb_cam_process_image_callback,
            queue_size=1
        )

        self._pub_face_coordinates = rospy.Publisher(
            'user/face_coordinates',
            Pose,
            queue_size=1)

    def validate_camera_resolution(self):
        # Check that the given camera resolution is valid for Baxter cameras
        if not any((self.camera_resolution[0] == r[0] and self.camera_resolution[1] == r[1]) for r in CameraController.MODES):
            rospy.logerr('ERROR: Invalid resolution provided.')
            print("ERROR: Invalid resolution provided.")

    def close_all_cameras(self):
        """
        Create the objects necessary to close all Baxter cameras in order to 
        only open the desired one in the <open_camera> method.
        """
        print("Closing all cameras...")
        cam = CameraController("head_camera")
        cam.close()
        cam = CameraController("right_hand_camera")
        cam.close()
        cam = CameraController("left_hand_camera")
        cam.close()

    def open_camera(self):
        """
        Open only the desired camera, based on the baxter_camera_name attribute.
        """
        print("Opening {} ...".format(self.baxter_camera_name))
        cam = CameraController(baxter_camera_name)
        cam.resolution = self.camera_resolution
        cam.open()

    def limb_cam_process_image_callback(self, ros_img):
        """
        This class is the middleware to receive the callbacks for the current 
        ros_img topic and enables the processing of that image so that the 
        Computer Vision algorithms can retrieve the current face position.
        :param ros_img: callback parameter given by Baxter's ROS built-in 
            topic for the /cameras/<desired_camera>/image .
        """
        image = cv_bridge.CvBridge().imgmsg_to_cv2(
            ros_img, desired_encoding="passthrough")

        # Apply face_detect algorithm to each image capture
        show_results = True
        fd = fdhc.FaceDetector(image, show_results, only_biggest_face=True)
        # print(fd.face_detect())

        faces = fd.face_detect()

        # Apply baxter_camera_point processing
        TM_w0_face = bcp.BaxterCameraCompleteTransform(
            faces, 1).get_tm_from_w0_to_face()

        self.current_coordinates = TM_w0_face[0:4, 3]
        print(self.current_coordinates)

        cv.waitKey(1)

    def execute_publish_coordinates(self):
        """
        Execute the publisher for the face coordinates in an infinite loop 
        based on the rospy_rate given in the __init__ method.
        """
        # Initial value until it is updated by a callback
        self.current_coordinates = np.array([0, 0, 0]).reshape((3, 1))

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

    main_node_face_coordinates = NodePublishFaceCoordinates((1280, 800), 10)

    main_node_face_coordinates.execute_publish_coordinates()

    rospy.on_shutdown(main_node_face_coordinates.close_all_cameras)

    cv.destroyAllWindows()
    return 0


if __name__ == '__main__':
    sys.exit(main())
