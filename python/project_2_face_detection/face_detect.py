# ATTEMPT FOR FACIAL DETECTION
# Santiago Garcia Arango
# Inspired by: https://realpython.com/face-recognition-with-python/


import cv2 as cv
import os

CURRENT_FOLDER = os.path.abspath(os.path.dirname(__file__))
print(CURRENT_FOLDER)

def main(image):
    # Create the haar cascade
    casc_path = os.path.join(CURRENT_FOLDER, "haarcascade_frontalface_default.xml")  # From OpenCV Github
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

    print(faces)
    print("Found {0} faces in given image!".format(len(faces)))

    # Draw a rectangle around the faces
    for (x, y, w, h) in faces:
        cv.rectangle(image, (x, y), (x + w, y + h), (255, 255, 55), 3)

    cv.imshow("FACES DETECTED", image)


if __name__ == "__main__":
    # TEST 1
    image_relative_path = os.path.join(CURRENT_FOLDER, "test_imgs", "picture0.jpg")
    image = cv.imread(image_relative_path)
    main(image)
    cv.waitKey(0)

    # TEST 2
    image_relative_path = os.path.join(CURRENT_FOLDER, "test_imgs", "picture1.png")
    image = cv.imread(image_relative_path)
    main(image)
    cv.waitKey(0)

    # TEST 3
    image_relative_path = os.path.join(CURRENT_FOLDER, "test_imgs", "picture2.png")
    image = cv.imread(image_relative_path)
    main(image)
    cv.waitKey(0)

    # TEST 4
    image_relative_path = os.path.join(CURRENT_FOLDER, "test_imgs", "picture3.png")
    image = cv.imread(image_relative_path)
    main(image)
    cv.waitKey(0)

    # TEST 5
    image_relative_path = os.path.join(CURRENT_FOLDER, "test_imgs", "picture4.png")
    image = cv.imread(image_relative_path)
    main(image)
    cv.waitKey(0)
