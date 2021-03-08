# FACE DETECT USING EXTERNAL LIBRARYE "FACE_RECOGNITION MODULE"

# Built-in imports
import os

# External libraries imports
import cv2 as cv
import numpy as np
import face_recognition


class FaceDetector:
    def __init__(self, image, show_results=False, only_biggest_face=False):
        self.image = image
        self.show_results = show_results
        self.only_biggest_face = only_biggest_face

    def face_detect(self):
        if self.image is None:
            print("ERROR: Image did not load correctly.")
            return False

        self.apply_face_recognition()

        # If self.faces returns a tuple, it means that no faces were found
        if (len(self.faces) == 0):
            if (self.show_results == True):
                print("Found 0 faces in given image!")
                cv.imshow("FACES DETECTED", self.image)
            return self.faces

        # Apply extra filter if only biggest face is desired
        if (self.only_biggest_face == True):
            self.faces = self.get_biggest_face()

        # Only draw face results if the show_results flag is activated
        if (self.show_results == True):
            # Draw a rectangle around the faces
            for (top, right, down, left) in self.faces:
                cv.rectangle(self.image, (left, top),
                             (right, down), (255, 255, 55), 3)
            cv.imshow("FACES DETECTED", self.image)
            print("Found faces correctly in given image!")

        return self.faces

    def apply_face_recognition(self):
        self.faces = face_recognition.face_locations(self.image)

    def get_biggest_face(self):
        # Create an array for the sizes (areas) of each of the faces
        array_of_sizes = []
        for (top, right, down, left) in self.faces:
            array_of_sizes.append(abs(top - down) * abs(right - left))

        # Get index for the biggest face based on the areas
        biggest_face_index = array_of_sizes.index(max(array_of_sizes))

        # Return a 2d array that contains only biggest face
        return np.reshape(self.faces[biggest_face_index], (-1, 4))


if __name__ == "__main__":
    print("\n--- LOOK FOR TESTS AND SAMPLES TO SEE HOW TO USE IT ---\n")
