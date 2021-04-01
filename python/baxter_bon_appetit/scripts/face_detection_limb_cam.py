#!/usr/bin/env python

# Built-int imports
import time

# Own imports
import face_detect_hc as fdhc

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


def open_camera(camera, resolution):
    if not any((resolution[0] == r[0] and resolution[1] == r[1]) for r in CameraController.MODES):
        rospy.logerr('Invalid resolution provided.')
    cam = CameraController(camera)
    cam.resolution = resolution
    cam.open()


def close_camera(camera):
    cam = CameraController(camera)
    cam.close()


def limb_cam_show_image_callback(ros_img):
    cv_image = cv_bridge.CvBridge().imgmsg_to_cv2(
        ros_img, desired_encoding="passthrough")
    cv.imshow('Image', cv_image)
    cv.waitKey(1)


def limb_cam_process_image_callback(ros_img):
    image = cv_bridge.CvBridge().imgmsg_to_cv2(
        ros_img, desired_encoding="passthrough")

    # Apply face_detect algorithm to each image capture
    show_results = True
    fd = fdhc.FaceDetector(image, show_results, only_biggest_face=True)
    print(fd.face_detect())

    cv.waitKey(1)


if __name__ == '__main__':
    rospy.init_node('Get_limb_camera_image', anonymous=True)
    rospy.Subscriber('/cameras/left_hand_camera/image',
                     Image, limb_cam_process_image_callback)
    rospy.spin()
    cv.destroyAllWindows()
