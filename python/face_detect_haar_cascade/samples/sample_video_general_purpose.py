# FACE DETECTION ON RECORDED VIDEO TO SEE FACE_DETECTION RESULTS

# Built-int imports
import os

# My own imports
import face_detect_hc as fdhc

# General module imports
import cv2 as cv
import time


class CorrectFramesCounter:
    def __init__(self, CAP, OUT):
        self.CAP = CAP
        self.OUT = OUT

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
                    fd = fdhc.FaceDetector(
                        image, show_results=True, only_biggest_face=True)
                    faces = fd.face_detect()

                    # Write video results in new video
                    self.OUT.write(image)

                    # Exit when "q" is pressed
                    if (cv.waitKey(1) & 0xFF == ord("q")):
                        break

                else:
                    break

        # When CAP is finished processing, close the capture object
        self.CAP.release()
        self.OUT.release()
        cv.destroyAllWindows()


if __name__ == "__main__":
    ROOT_FOLDER = os.path.abspath(os.path.join(
        os.path.dirname(__file__), os.path.pardir))

    CAP = cv.VideoCapture(os.path.join(
        ROOT_FOLDER, "assets", "videos", "sample_video_multiple_faces_0.mp4"))

    fourcc = cv.VideoWriter_fourcc(*'MJPG')
    OUT = cv.VideoWriter("sample_video_multiple_faces_0_results.avi", fourcc,
                         20.0, (int(CAP.get(3)), int(CAP.get(4))))

    # Run main frame analyzer
    check_results = CorrectFramesCounter(CAP, OUT)
    check_results.main()

    # Gather frame analyzer results
    total = check_results.total_frames
    correct = check_results.correct_frames
    success = str((float(correct) / float(total)) * 100)
