# import modules
import cv2
import numpy as np
import imutils

# define ranges of spectrum in HSV[] for each object
# Ball=============================================
lower_ball = np.array([0, 122, 91])
upper_ball = np.array([255, 250, 255])

BALL_WIDTH = 0.043  # m
FOCAL_LENGTH = 123.423  # m
DEG_PER_PX = 0.0192

# Begin video capture
cap = cv2.VideoCapture(0)  # Connect to camera 0 (or the only camera)
cap.set(3, 320)  # Set the width to 320
cap.set(4, 240)  # Set the height to 240


def distance_to_camera(knownWidth, focalLength, perWidth):
    # compute and return the distance from the maker to the camera
    return (knownWidth * focalLength) / perWidth


def distance(image):
    # find the contours in the edged image and keep the largest one
    contours = cv2.findContours(image.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    contours = imutils.grab_contours(contours)
    if len(contours) > 0:
        c = max(contours, key=cv2.contourArea)

        marker = cv2.minAreaRect(c)

        dist = distance_to_camera(BALL_WIDTH, FOCAL_LENGTH, marker[1][0])
        print("Distance to Ball: " + str(dist))

        box = cv2.cv.BoxPoints(marker) if imutils.is_cv2() else cv2.boxPoints(marker)
        box = np.int0(box)
        cv2.drawContours(frame, [box], -1, (0, 255, 0), 2)
        cv2.putText(frame, "%.2fm" % dist,
                    (frame.shape[1] - 135, frame.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX,
                    1, (0, 255, 0), 2)


def bearing_to_camera(position, degreePixelRatio):
    # compute and return the distance from the maker to the camera
    return position * degreePixelRatio


def bearing(image):
    # find the contours in the edged image and keep the largest one
    contours = cv2.findContours(image.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    contours = imutils.grab_contours(contours)
    if len(contours) > 0:
        c = max(contours, key=cv2.contourArea)

        marker = cv2.minAreaRect(c)
        angle = bearing_to_camera(marker[0][1], DEG_PER_PX)
        print("Angle to Ball: " + str(bearing))

        box = cv2.cv.BoxPoints(marker) if imutils.is_cv2() else cv2.boxPoints(marker)
        box = np.int0(box)

        cv2.putText(frame, "%.2fdeg" % angle,
                    (frame.shape[1] - 135, frame.shape[0] - 40), cv2.FONT_HERSHEY_SIMPLEX,
                    1, (0, 255, 0), 2)


while True:
    # capture each frame
    ret, frame = cap.read()
    # blur frames with gaussian 5x5 kernel and determined std dev
    blur = cv2.GaussianBlur(frame, (5, 5), 0)
    # Convert BGR to HSV
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

    # Threshold the HSV image and erode to create clear mask
    ball_mask = cv2.inRange(hsv, lower_ball, upper_ball)
    ball_mask = cv2.erode(ball_mask, None, iterations=2)

    # Bitwise-AND mask and original image
    masked = cv2.bitwise_and(frame, frame, mask=ball_mask)

    # detect edges using Canny detect
    edged = cv2.Canny(masked, 35, 125)

    # =================================================================
    distance(edged)
    bearing(edged)
    # =================================================================

    # Display steps of masking images as video
    cv2.imshow('frame', frame)
    # cv2.imshow('blur', blur)
    # cv2.imshow('individual mask', ball_mask)
    cv2.imshow('masked', masked)
    cv2.imshow('edged', edged)

    k = cv2.waitKey(5) & 0xFF
    # k == 27 is for Esc key
    if k == 27:
        break
    # cv2.waitKey(0)  # Exit on keypress

cap.release()  # Release the camera object
cv2.destroyAllWindows()  # stops program
