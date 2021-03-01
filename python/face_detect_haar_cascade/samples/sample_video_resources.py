# FACE DETECTION ON RECORDED VIDEO TO CHECK TOTAL RESOURCES

# Built-int imports
import os
import math

# My own imports
import face_detect_hc as fdhc

# General module imports
import cv2 as cv
import numpy as np
import time
import psutil


class TotalResourcesAnalyzer:
    def __init__(self, CAP):
        self.CAP = CAP
        self.cpu = []
        self.memory = []

    def main(self):
        # check_results if video loaded correctly
        if (self.CAP.isOpened() == False):
            print("Error opening video stream or file")
        else:
            while (self.CAP.isOpened()):
                # Get specific read image from each instant from CAP capture
                ret, image = CAP.read()

                if ret == True:
                    # Apply face_detect algorithm to each image capture
                    fd = fdhc.FaceDetector(image, show_results=True, only_biggest_face=True)
                    faces = fd.face_detect()

                    self.cpu.append(psutil.cpu_percent(interval=None))
                    self.memory.append(psutil.swap_memory()[3])

                    # Exit when "q" is pressed
                    if (cv.waitKey(1) & 0xFF == ord("q")):
                        break

                else:
                    break

        # When CAP is finished processing, close the capture object
        self.CAP.release()
        cv.destroyAllWindows()

    def plot_resources(self):
        print(" ------ RESULTS -----")
        print("--> CPU PERCENTAGE:")
        print(len(self.cpu))
        print(self.cpu)
        print(np.mean(self.cpu))
        print("--> MEMORY PERCENTAGE:")
        print(len(self.memory))
        print(self.memory)
        print(np.mean(self.memory))

if __name__ == "__main__":
    ROOT_FOLDER = os.path.abspath(os.path.join(
        os.path.dirname(__file__), os.path.pardir))

    CAP = cv.VideoCapture(os.path.join(
        ROOT_FOLDER, "assets", "videos", "sample_video_0.mp4"))

    # Run main frame analyzer
    check_results = TotalResourcesAnalyzer(CAP)
    check_results.main()
    check_results.plot_resources()
