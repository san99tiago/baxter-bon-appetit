# Test framework
import pytest

# Built-int imports
import os

# External imports
import cv2 as cv
import numpy as np

# My own imports
import face_detect_hc as fdfr
import get_assets_folder as gaf

# Get assets folder in repo for the samples
ASSETS_FOLDER = gaf.get_assets_folder_path()


class TestProcessImage:
    def test_process_image_without_faces(self):
        """
        Test that an real opened image without faces doen't return faces
        """
        im_path = os.path.abspath(os.path.join(
            ASSETS_FOLDER, "imgs", "nofaces_0.jpg"))
        real_image = cv.imread(im_path)
        fd = fdfr.FaceDetector(
            real_image, show_results=False, only_biggest_face=True)
        faces = fd.face_detect()
        assert (type(faces) is tuple)

    def test_process_image_with_one_face(self):
        """
        Test that an real opened image with one face, returns one face
        """
        im_path = os.path.abspath(os.path.join(
            ASSETS_FOLDER, "imgs", "faces_0.jpg"))
        real_image = cv.imread(im_path)
        fd = fdfr.FaceDetector(
            real_image, show_results=False, only_biggest_face=True)
        faces = fd.face_detect()
        if (type(faces) is tuple):
            AssertionError

        assert len(faces) == 1

    def test_process_image_with_two_faces(self):
        """
        Test that an real opened image with two face, returns two faces
        """
        im_path = os.path.abspath(os.path.join(
            ASSETS_FOLDER, "imgs", "faces_1.jpg"))
        real_image = cv.imread(im_path)
        fd = fdfr.FaceDetector(
            real_image, show_results=False, only_biggest_face=False)
        faces = fd.face_detect()
        if (type(faces) is tuple):
            AssertionError

        assert len(faces) == 2


if __name__ == '__main__':
    tests = TestProcessImage()
    tests.test_process_image_without_faces()
    tests.test_process_image_with_one_face()
    tests.test_process_image_with_two_faces()