# FACE DETECTION ON RECORDED VIDEO TO CHECK TOTAL RESOURCES

# Built-int imports
import os
import math
import time


# My own imports
import face_detect_hc as fdhc

# General module imports
import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
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
        # Show general results
        print(" ------ RESULTS -----")
        print("--> CPU PERCENTAGE:")
        print(len(self.cpu))
        print(self.cpu)
        print(np.mean(self.cpu))
        print("--> MEMORY PERCENTAGE:")
        print(len(self.memory))
        print(self.memory)
        print(np.mean(self.memory))

        # Generate plots for resources
        t_1 = np.linspace(0, 1, len(self.cpu))
        figure_1 = plt.figure(1)
        axes_1 = figure_1.add_axes([0.1, 0.1, 0.8, 0.8])
        axes_1.plot(t_1, self.cpu, c='b', linewidth=3)
        axes_1.set_title("CPU PERCENTAGE USED")
        axes_1.set_xlabel("Time")
        axes_1.set_ylabel("CPU percentage")
        axes_1.set_ylim([0, 100])
        figure_1.patch.set_facecolor((0.2, 1, 1))

        figure_2 = plt.figure(2)
        axes_2 = figure_2.add_axes([0.1, 0.1, 0.8, 0.8])
        axes_2.plot(t_1, self.memory, c='r', linewidth=3)
        axes_2.set_title("MEMORY PERCENTAGE USED")
        axes_2.set_xlabel("Time")
        axes_2.set_ylabel("MEMORY percentage")
        axes_2.set_ylim([0, 100])
        figure_2.patch.set_facecolor((0.2, 1, 1))
        plt.show()

if __name__ == "__main__":
    ROOT_FOLDER = os.path.abspath(os.path.join(
        os.path.dirname(__file__), os.path.pardir))

    CAP = cv.VideoCapture(os.path.join(
        ROOT_FOLDER, "assets", "videos", "sample_video_0.mp4"))

    # Run main frame analyzer
    check_results = TotalResourcesAnalyzer(CAP)
    check_results.main()
    check_results.plot_resources()
