
# General module imports
import cv2 as cv
import os
import face_recognition

CURRENT_FOLDER = os.path.abspath(os.path.dirname(__file__))

def main(image):
    if image is None:
        print("ERROR: Image did not load correctly.")
        return False

    faces = face_recognition.face_locations(image)

    print(faces)
    print("Found {0} faces in given image!".format(len(faces)))

    # Draw a rectangle around the faces
    for (top, right, down, left) in faces:
        cv.rectangle(image, (left, top), (right, down), (255, 255, 55), 3)

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

