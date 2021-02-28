# MULTIPLE IMAGE SAMPLES FOR FACE_DETECTION WITH HAAR_CASCADE APPROACH

# Built-int imports
import os

# External imports
import cv2 as cv
import numpy as np

# My own imports
import face_detect_hc as fdhc

ROOT_FOLDER = os.path.abspath(os.path.join(
    os.path.dirname(__file__), os.path.pardir))

def main():
    # Sample 1
    image_relative_path = os.path.join(
        ROOT_FOLDER, "assets", "imgs", "faces_0.jpg")
    image = cv.imread(image_relative_path)
    fdhc.face_detect(image)
    cv.waitKey(0)

    # Sample 2
    image_relative_path = os.path.join(
        ROOT_FOLDER, "assets", "imgs", "faces_1.jpg")
    image = cv.imread(image_relative_path)
    fdhc.face_detect(image)
    cv.waitKey(0)

    # Sample 3
    image_relative_path = os.path.join(
        ROOT_FOLDER, "assets", "imgs", "faces_2.jpg")
    image = cv.imread(image_relative_path)
    fdhc.face_detect(image)
    cv.waitKey(0)

    # Sample 4
    image_relative_path = os.path.join(
        ROOT_FOLDER, "assets", "imgs", "faces_3.jpg")
    image = cv.imread(image_relative_path)
    fdhc.face_detect(image)
    cv.waitKey(0)

    # Sample 5
    image_relative_path = os.path.join(
        ROOT_FOLDER, "assets", "imgs", "faces_4.jpg")
    image = cv.imread(image_relative_path)
    fdhc.face_detect(image)
    cv.waitKey(0)

    # Sample 6
    image_relative_path = os.path.join(
        ROOT_FOLDER, "assets", "imgs", "faces_5.jpg")
    image = cv.imread(image_relative_path)
    fdhc.face_detect(image)
    cv.waitKey(0)

    # Sample 7
    image_relative_path = os.path.join(
        ROOT_FOLDER, "assets", "imgs", "nofaces_0.jpg")
    image = cv.imread(image_relative_path)
    fdhc.face_detect(image)
    cv.waitKey(0)


if __name__ == "__main__":
    main()
