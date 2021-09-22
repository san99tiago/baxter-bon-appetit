# MULTIPLE IMAGE SAMPLES FOR FACE_DETECTION WITH HAAR_CASCADE APPROACH

# Built-int imports
import os
import glob

# External imports
import cv2 as cv
import numpy as np

# My own imports
import face_detect_hc as fdhc
import get_assets_folder as gaf

# Get assets folder in repo for the samples
ASSETS_FOLDER = gaf.get_assets_folder_path()

print(ASSETS_FOLDER)
RESULTS_FOLDER = os.path.join(ASSETS_FOLDER, "imgs_different_people_detected")

def main(save_data):

    all_photos = glob.glob(os.path.join(ASSETS_FOLDER, "imgs_different_people", "*.png"))

    cont = 0
    for photo in all_photos:
        cont = cont + 1
        try:
            image = cv.imread(photo)
            image = cv.resize(image, (500, 700))
            fd = fdhc.FaceDetector(image, show_results=True, only_biggest_face=True)
            fd.face_detect()

            if (save_data == "yes"):
                path_to_save_photo = os.path.join(RESULTS_FOLDER, "person_{}.png".format(str(cont)))
                cv.imwrite(path_to_save_photo, image)

            cv.waitKey(0)
        except:
            print("There was an error opening photo: " + photo)


if __name__ == "__main__":
    main("no")
