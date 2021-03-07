# ATTEMPT FOR FACIAL DETECTION
# Santiago Garcia Arango
# Inspired by: https://realpython.com/face-recognition-with-python/

# Built-in imports
import os

# External libraries imports
import cv2 as cv
import numpy as np

ROOT_FOLDER = os.path.abspath(os.path.join(
    os.path.dirname(__file__), os.path.pardir))
print("ROOT_FOLDER --> ", ROOT_FOLDER)


class FaceDetector:
    def __init__(self, image, show_results=False, only_biggest_face=False):
        self.image = image
        self.show_results = show_results
        self.only_biggest_face = only_biggest_face

    def face_detect(self):
        if self.image is None:
            print("ERROR: Image did not load correctly.")
            return False

        self.generate_haar_cascade_clasifier()
        self.apply_haar_cascade_clasifier()

        # If self.faces returns a tuple, it means that no faces were found
        if (type(self.faces) is tuple):
            if (self.show_results == True):
                print("Found 0 faces in given image!")
                cv.imshow("FACES DETECTED", self.image)
            return self.faces

        # Apply extra filter if only biggest face is desired
        if (self.only_biggest_face == True):
            self.faces = self.get_biggest_face()

        # Only draw face results if the show_results flag is activated
        if (self.show_results == True):
            # Draw a rectangle in the face/faces found
            for (x, y, w, h) in self.faces:
                cv.rectangle(self.image, (x, y),
                             (x + w, y + h), (255, 255, 55), 3)
            cv.imshow("FACES DETECTED", self.image)
            print("Found faces correctly in given image!")

        return self.faces

    def generate_haar_cascade_clasifier(self):
        # Create the haar cascade
        casc_path = os.path.join(
            ROOT_FOLDER, "src", "haarcascade_frontalface_default.xml")
        self.face_cascade = cv.CascadeClassifier(casc_path)

    def apply_haar_cascade_clasifier(self):
        gray_image = cv.cvtColor(self.image, cv.COLOR_BGR2GRAY)

        # Detect faces in the image (vector of faces with rectangle dimensions)
        self.faces = self.face_cascade.detectMultiScale(
            gray_image,
            # scaleFactor=1.15,
            scaleFactor=1.10,
            minNeighbors=5,
            minSize=(30, 30),
            flags=cv.CASCADE_SCALE_IMAGE
        )

    def get_biggest_face(self):
        # Create an array for the sizes (areas) of each of the faces
        array_of_sizes = []
        for (x, y, w, h) in self.faces:
            array_of_sizes.append(w * h)

        # Get index for the biggest face based on the areas
        biggest_face_index = array_of_sizes.index(max(array_of_sizes))

        # Return a 2d array that contains only biggest face
        return np.reshape(self.faces[biggest_face_index], (-1, 4))


if __name__ == "__main__":
    print("--- LOOK FOR TESTS AND SAMPLES TO SEE HOW TO USE IT ---")

    im_path = os.path.abspath(os.path.join(
        ROOT_FOLDER, "assets", "imgs", "faces_1.jpg"))
    real_image = cv.imread(im_path)
    fd = FaceDetector(real_image, show_results=True, only_biggest_face=True)
    print(fd.face_detect())
    cv.waitKey(0)
