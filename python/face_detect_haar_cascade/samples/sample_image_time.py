# MULTIPLE IMAGE SAMPLES FOR FACE_DETECTION WITH HAAR_CASCADE APPROACH

# Built-int imports
import os
import time

# External imports
import cv2 as cv
import numpy as np

# My own imports
import face_detect_hc as fdhc

ROOT_FOLDER = os.path.abspath(os.path.join(
    os.path.dirname(__file__), os.path.pardir))

def process_one_image_face():
    # Sample 1
    image_relative_path = os.path.join(
        ROOT_FOLDER, "assets", "imgs", "faces_0.jpg")
    image = cv.imread(image_relative_path)
    fdhc.face_detect(image)
    cv.destroyAllWindows()

if __name__ == "__main__":
    start_time= time.time()
    process_one_image_face()
    print(" ---- %s seconds ----" % (time.time() - start_time))
