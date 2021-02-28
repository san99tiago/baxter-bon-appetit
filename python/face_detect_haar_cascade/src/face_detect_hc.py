# ATTEMPT FOR FACIAL DETECTION
# Santiago Garcia Arango
# Inspired by: https://realpython.com/face-recognition-with-python/


import cv2 as cv
import os

ROOT_FOLDER = os.path.abspath(os.path.join(
    os.path.dirname(__file__), os.path.pardir))
print("ROOT_FOLDER --> ", ROOT_FOLDER)


def face_detect(image):
    # Create the haar cascade
    casc_path = os.path.join(
        ROOT_FOLDER, "src", "haarcascade_frontalface_default.xml")
    faceCascade = cv.CascadeClassifier(casc_path)

    if image is None:
        print("ERROR: Image did not load correctly.")
        return False

    gray_image = cv.cvtColor(image, cv.COLOR_BGR2GRAY)

    # Detect faces in the image (vector of faces with rectangle dimensions)
    faces = faceCascade.detectMultiScale(
        gray_image,
        scaleFactor=1.15,
        minNeighbors=5,
        minSize=(30, 30),
        flags=cv.CASCADE_SCALE_IMAGE
    )

    print("Found {0} faces in given image!".format(len(faces)))

    # Draw a rectangle around the faces
    for (x, y, w, h) in faces:
        cv.rectangle(image, (x, y), (x + w, y + h), (255, 255, 55), 3)

    cv.imshow("FACES DETECTED", image)

    return faces


if __name__ == "__main__":
    print("LOOK FOR TESTS AND SAMPLES TO SEE HOW TO USE IT")
