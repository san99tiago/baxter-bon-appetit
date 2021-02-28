# Test framework
import pytest

# Built-int imports
import os

# External imports
import cv2 as cv
import numpy as np

# My own imports
import face_detect_hc as fdhc

ROOT_FOLDER = os.path.abspath(os.path.join(
    os.path.dirname(__file__), os.path.pardir))


class TestProcessImage:
    def test_process_image_without_faces(self):
        """
        Test that an real opened image without faces doen't return faces
        """
        im_path = os.path.abspath(os.path.join(
            ROOT_FOLDER, "assets", "imgs", "nofaces_0.jpg"))
        real_image = cv.imread(im_path)
        faces = fdhc.face_detect(real_image)
        assert len(faces) == 0

    def test_process_image_with_one_face(self):
        """
        Test that an real opened image with one face, returns one face
        """
        im_path = os.path.abspath(os.path.join(
            ROOT_FOLDER, "assets", "imgs", "faces_0.jpg"))
        real_image = cv.imread(im_path)
        faces = fdhc.face_detect(real_image)
        assert len(faces) == 1

    def test_process_image_with_two_faces(self):
        """
        Test that an real opened image with two face, returns two faces
        """
        im_path = os.path.abspath(os.path.join(
            ROOT_FOLDER, "assets", "imgs", "faces_1.jpg"))
        real_image = cv.imread(im_path)
        faces = fdhc.face_detect(real_image)
        assert len(faces) == 2


if __name__ == '__main__':
    tests = TestProcessImage()
    tests.test_process_image_without_faces()
    tests.test_process_image_with_one_face()
    tests.test_process_image_with_two_faces()
