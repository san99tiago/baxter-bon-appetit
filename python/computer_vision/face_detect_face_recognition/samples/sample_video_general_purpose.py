# FACE DETECTION ON RECORDED VIDEO TO SEE FACE_DETECTION RESULTS

# Built-int imports
import os

# My own imports
import face_detect_fr as fdfr
import get_assets_folder as gaf

# General module imports
import cv2 as cv


class CorrectFramesCounter:
    def __init__(self, CAP, OUT):
        self.CAP = CAP
        self.OUT = OUT

    def main(self):
        # check_results if video loaded correctly
        if (self.CAP.isOpened() == False):
            print("Error opening video stream or file")
            return False
        else:
            while (self.CAP.isOpened()):
                # Get specific read image from each instant from CAP capture
                ret, image = CAP.read()

                if ret == True:
                    # Apply face_detect algorithm to each image capture
                    fd = fdfr.FaceDetector(
                        image, show_results=True, only_biggest_face=True)
                    faces = fd.face_detect()

                    # Write video results in new video
                    self.OUT.write(image)

                    # Exit when "q" is pressed
                    if (cv.waitKey(1) & 0xFF == ord("q")):
                        return True

                else:
                    return True

        # When CAP is finished processing, close the capture object
        self.CAP.release()
        self.OUT.release()
        cv.destroyAllWindows()


if __name__ == "__main__":
    # Get assets folder in repo for the samples
    ASSETS_FOLDER = gaf.get_assets_folder_path()

    CAP = cv.VideoCapture(os.path.join(
        ASSETS_FOLDER, "videos", "sample_video_multiple_faces_0.mp4"))

    fourcc = cv.VideoWriter_fourcc(*'MJPG')
    OUT = cv.VideoWriter("sample_video_multiple_faces_0_output_fdfr.avi", fourcc,
                         20.0, (int(CAP.get(3)), int(CAP.get(4))))

    # Run main frame analyzer
    check_results = CorrectFramesCounter(CAP, OUT)
    check_results.main()
