# import the necessary packages
from imutils import paths
import numpy as np
import imutils
import cv2


def find_marker(image):
    # convert the image to grayscale, blur it, and detect edges
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    edged = cv2.Canny(gray, 35, 125)

    # find the contours in the edged image and keep the largest one;
    # assume that this is our piece of paper in the image
    cnts = cv2.findContours(edged.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    c = max(cnts, key=cv2.contourArea)

    # compute the bounding box of the of the paper region and return it
    return cv2.minAreaRect(c)


def distance_to_camera(knownWidth, focalLength, perWidth):
    # compute and return the distance from the maker to the camera
    return (knownWidth * focalLength) / perWidth


# initialize the known distance from the camera to the object(m)
KNOWN_DISTANCE = 0.6

# initialize the known object width (m)
KNOWN_WIDTH = 0.043

lower_ball = np.array([0, 122, 91])
upper_ball = np.array([255, 250, 255])

# load the first image that contains an object, then find the
# ball marker in the image, and initialize the focal length
image = cv2.imread("distTest30.png")
blur = cv2.GaussianBlur(image, (5, 5), 0)
hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
# Threshold the HSV image and erode to create clear mask
ball_mask = cv2.inRange(hsv, lower_ball, upper_ball)
ball_mask = cv2.erode(ball_mask, None, iterations=2)
masked = cv2.bitwise_and(image, image, mask=ball_mask)
marker = find_marker(masked)
focalLength = (marker[1][0] * KNOWN_DISTANCE) / KNOWN_WIDTH
print("Focal length: " + str(focalLength))

mm = distance_to_camera(KNOWN_WIDTH, focalLength, marker[1][0])

# draw a bounding box around the image and display it
box = cv2.cv.BoxPoints(marker) if imutils.is_cv2() else cv2.boxPoints(marker)
box = np.int0(box)
cv2.drawContours(image, [box], -1, (0, 255, 0), 2)
cv2.putText(image, "%.2fm" % mm,
            (image.shape[1] - 200, image.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX,
            2.0, (0, 255, 0), 3)
cv2.imshow("image", image)
cv2.waitKey(0)
