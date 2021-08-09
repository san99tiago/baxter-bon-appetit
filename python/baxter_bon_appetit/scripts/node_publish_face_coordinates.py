#!/usr/bin/env python

# Built-in imports
import time
import sys
import numpy as np
import socket

# Own imports
import face_detect_hc as fdhc
import baxter_vision_mapping.baxter_camera_complete_transform as bcp

# General module imports
import rospy
import rosgraph
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

from baxter_core_msgs.srv import (
    ListCameras,
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
        self.show_open_publishing_cameras()
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

    def show_open_publishing_cameras(self):
        """
        Create the objects necessary to close all Baxter cameras in order to 
        only open the left limb one. This method ensures that the other cameras 
        are closed before opening the left camera.
        Remark: we got inspired from the original "baxter_tools" scripts.
        """
        ls = rospy.ServiceProxy('cameras/list', ListCameras)
        rospy.wait_for_service('cameras/list', timeout=10)
        resp = ls()
        if len(resp.cameras):
            # Find open (publishing) cameras
            master = rosgraph.Master('/rostopic')
            resp.cameras
            cam_topics = dict([(cam, "/cameras/%s/image" % cam)
                               for cam in resp.cameras])
            self.open_cams = dict([(cam, False) for cam in resp.cameras])
            try:
                topics = master.getPublishedTopics('')
                for topic in topics:
                    for cam in resp.cameras:
                        if topic[0] == cam_topics[cam]:
                            self.open_cams[cam] = True
            except socket.error:
                raise ROSTopicIOException("Cannot communicate with master.")
            for cam in resp.cameras:
                print("%s%s" %
                      (cam, ("  -  (open)" if self.open_cams[cam] else "")))

            print(self.open_cams)
        else:
            print('No cameras found')

    def open_camera(self):
        """
        Based on the open cameras, analyze if the left-hand camera is opened or 
        not. The purpose of this method, is to guarantee that this camera will 
        always be anabled before the image-processing.
        """

        # Flag to verify that left_hand_camera exists in the open_cams register
        left_hand_camera_exists = False

        # Open left_hand_camera in case that it exists in the register
        for cam in self.open_cams:
            if (cam == "left_hand_camera"):
                left_hand_camera_exists = True
                if (self.open_cams["left_hand_camera"] == False):
                    CameraController("left_hand_camera").open()

        # Open left_hand_camera if it does not exist in register
        if (left_hand_camera_exists == False):
            CameraController("head_camera").close()
            CameraController("left_hand_camera").open()

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
            faces, 0.5).get_tm_from_w0_to_face()

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

    main_node_face_coordinates = NodePublishFaceCoordinates((640, 400), 100)

    main_node_face_coordinates.execute_publish_coordinates()

    rospy.on_shutdown(main_node_face_coordinates.show_open_publishing_cameras)

    cv.destroyAllWindows()
    return 0


if __name__ == '__main__':
    sys.exit(main())
