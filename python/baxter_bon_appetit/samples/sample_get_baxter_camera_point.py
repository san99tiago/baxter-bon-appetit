#!/usr/bin/env python
# SAMPLE OF FACE_DETECT_HAAR_CASCADE WITH BAXTER_CAMERA_POINT

# Own imports
import face_detect_hc as fdhc
import baxter_vision_mapping.baxter_camera_complete_transform as bcp

# General module imports
import cv2 as cv
import time

VIDEO_CAPTURE = cv.VideoCapture(0)

while True:
    # Get specific read image from each instant from video capture
    _, image = VIDEO_CAPTURE.read()

    # Apply face_detect algorithm to each image capture
    fd = fdhc.FaceDetector(image, show_results=True, only_biggest_face=True)
    faces = fd.face_detect()

    print(faces)

    # Apply baxter_camera_point processing
    TM_w0_face = bcp.BaxterCameraCompleteTransform(faces, 1).get_tm_from_w0_to_face()

    # Apply delay to obtain desired frequency
    time.sleep(0.1)

    # Exit when "q" is pressed
    if (cv.waitKey(1) & 0xFF == ord("q")):
        cv.destroyAllWindows()
        break
