# MULTIPLE IMAGE SAMPLES FOR FACE_DETECTION WITH HAAR_CASCADE APPROACH

# Built-int imports
import os

# External imports
import cv2 as cv
import numpy as np

# My own imports
import face_detect_hc as fdhc
import get_assets_folder as gaf

# Get assets folder in repo for the samples
ASSETS_FOLDER = gaf.get_assets_folder_path()


def main():

    for i in range(25):
        # Sample "i"
        image_relative_path = os.path.join(
            ASSETS_FOLDER, "imgs_different_people", "person_" + str(i + 1) + ".png")
        image = cv.imread(image_relative_path)
        image = cv.resize(image, (500, 700))
        fd = fdhc.FaceDetector(image, show_results=True, only_biggest_face=True)
        fd.face_detect()
        cv.waitKey(0)


if __name__ == "__main__":
    main()
