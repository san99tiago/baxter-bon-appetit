# MULTIPLE IMAGE SAMPLES FOR FACE_DETECTION WITH HAAR_CASCADE APPROACH

# Built-int imports
import os
import time

# External imports
import cv2 as cv
import numpy as np

# My own imports
import face_detect_hc as fdhc
import get_assets_folder as gaf

def process_one_image_face():
    # Sample 1
    image_relative_path = os.path.join(
        ASSETS_FOLDER, "assets", "imgs", "faces_0.jpg")
    image = cv.imread(image_relative_path)
    fd = fdhc.FaceDetector(image, show_results=False, only_biggest_face=False)
    fd.face_detect()
    cv.destroyAllWindows()


if __name__ == "__main__":
    # Get assets folder in repo for the samples
    ASSETS_FOLDER = gaf.get_assets_folder_path()

    start_time = time.time()
    process_one_image_face()
    print(" ---- %s seconds ----" % (time.time() - start_time))
