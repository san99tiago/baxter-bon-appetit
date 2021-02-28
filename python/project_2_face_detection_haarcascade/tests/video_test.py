# FACIAL DETECTION ON VIDEO
# Santiago Garcia Arango

# My own imports
import face_detect

# General module imports
import cv2 as cv
import time

VIDEO_CAPTURE = cv.VideoCapture(0)

while True:
    # Get specific read image from each instant from video capture
    _, image = VIDEO_CAPTURE.read()

    # Apply face_detect algorithm to each image capture
    face_detect.main(image)

    # Apply delay to obtain desired frequency
    time.sleep(0.1)

    # Exit when "q" is pressed
    if (cv.waitKey(1) & 0xFF == ord("q")):
        cv.destroyAllWindows()
        break
