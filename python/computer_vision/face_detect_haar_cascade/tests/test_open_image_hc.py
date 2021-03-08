# Test framework
import pytest

# Built-int imports
import os

# External imports
import cv2 as cv
import numpy as np

# My own imports
import get_assets_folder as gaf

# Get assets folder in repo for the samples
ASSETS_FOLDER = gaf.get_assets_folder_path()


class TestOpenImage:
    def test_empty_open_image(self):
        """
        Test that an empty opened image must return string message
        """
        empty_image = cv.imread("notExistingImage.jpg")
        assert empty_image == None

    def test_real_open_image(self):
        """
        Test that an real opened image must return correct size
        """
        im_path = os.path.abspath(os.path.join(
            ASSETS_FOLDER, "imgs", "faces_0.jpg"))
        # print(im_path)
        real_image = cv.imread(im_path)
        # print(real_image)
        assert real_image is not None


if __name__ == '__main__':
    tests = TestOpenImage()
    tests.test_empty_open_image()
    tests.test_real_open_image()
