# import modules
import cv2
import numpy as np
import imutils
import time

# define ranges of spectrum in HSV[] for each object
# Ball=============================================
lower_ball = np.array([0, 135, 150])
upper_ball = np.array([255, 250, 255])
lower_obstacle = np.array([0, 0, 0])
upper_obstacle = np.array([180, 110, 54])

BALL_WIDTH = 0.043  # m
OBSTACLE_WIDTH = 0.18    # m
FOCAL_LENGTH = 123.423  # m
DEG_PER_PX = 0.0192

# Begin video capture
cap = cv2.VideoCapture(0)  # Connect to camera 0 (or the only camera)
cap.set(3, 320)  # Set the width to 320
cap.set(4, 240)  # Set the height to 240

# ================================================================

# =============================================


def DistanceToCamera(knownWidth, focalLength, perWidth):
    # compute and return the distance from the maker to the camera
    if perWidth != 0:
        return (knownWidth * focalLength) / perWidth
    else:
        return 0


def DistanceB(image):
    # find the contours in the edged image and keep the largest one
    contours = cv2.findContours(image.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    contours = imutils.grab_contours(contours)
    if len(contours) > 0:
        c = max(contours, key=cv2.contourArea)

        marker = cv2.minAreaRect(c)

        dist_ball = DistanceToCamera(BALL_WIDTH, FOCAL_LENGTH, marker[1][0])
        print("Distance to Ball: " + str(dist_ball))

        box = cv2.cv.BoxPoints(marker) if imutils.is_cv2() else cv2.boxPoints(marker)
        box = np.int0(box)
        cv2.drawContours(frame, [box], -1, (0, 255, 0), 2)
        cv2.putText(frame, "b: %.2fm" % dist_ball,
                    (frame.shape[1] - 100, frame.shape[0] - 100), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 0), 2)


def DistanceO(image):
    # find the contours in the edged image and keep the largest one
    contours = cv2.findContours(image.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    contours = imutils.grab_contours(contours)
    if len(contours) > 0:
        c = max(contours, key=cv2.contourArea)

        marker = cv2.minAreaRect(c)

        dist_obstacle = DistanceToCamera(OBSTACLE_WIDTH, FOCAL_LENGTH, marker[1][0])
        print("Distance to Obstacle: " + str(dist_obstacle))

        box = cv2.cv.BoxPoints(marker) if imutils.is_cv2() else cv2.boxPoints(marker)
        box = np.int0(box)
        cv2.drawContours(frame, [box], -1, (0, 255, 0), 2)
        cv2.putText(frame, "o: %.2fm" % dist_obstacle,
                    (frame.shape[1] - 100, frame.shape[0] - 40), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 0), 2)


def BearingToCamera(position, degreePixelRatio):
    # compute and return the distance from the maker to the camera
    return position * degreePixelRatio


def BearingB(image):
    # find the contours in the edged image and keep the largest one
    contours = cv2.findContours(image.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    contours = imutils.grab_contours(contours)
    if len(contours) > 0:
        c = max(contours, key=cv2.contourArea)

        marker = cv2.minAreaRect(c)
        angle_obstacle = BearingToCamera(marker[1][0], DEG_PER_PX)
        print("Angle to Ball: " + str(angle_obstacle))

        box = cv2.cv.BoxPoints(marker) if imutils.is_cv2() else cv2.boxPoints(marker)
        box = np.int0(box)

        cv2.putText(frame, "b: %.2fdeg" % angle_obstacle,
                    (frame.shape[1] - 100, frame.shape[0] - 70), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 0), 2)


def BearingO(image):
    # find the contours in the edged image and keep the largest one
    contours = cv2.findContours(image.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    contours = imutils.grab_contours(contours)
    if len(contours) > 0:
        c = max(contours, key=cv2.contourArea)

        marker = cv2.minAreaRect(c)
        angle_ball = BearingToCamera(marker[1][0], DEG_PER_PX)
        print("Angle to Obstacle: " + str(angle_ball))

        box = cv2.cv.BoxPoints(marker) if imutils.is_cv2() else cv2.boxPoints(marker)
        box = np.int0(box)

        cv2.putText(frame, "o: %.2fdeg" % angle_ball,
                    (frame.shape[1] - 100, frame.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 0), 2)


while True:
    startTime = time.time()
    # capture each frame
    ret, frame = cap.read()
    # Flip image
    # frame = cv2.flip(frame, 0)
    # blur frames with gaussian 5x5 kernel and determined std dev
    blur = cv2.GaussianBlur(frame, (5, 5), 0)
    # Convert BGR to HSV
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

    # Threshold the HSV image and erode to create clear mask
    ball_mask = cv2.inRange(hsv, lower_ball, upper_ball)
    ball_mask = cv2.erode(ball_mask, None, iterations=2)

    obstacle_mask = cv2.inRange(hsv, lower_obstacle, upper_obstacle)
    obstacle_mask = cv2.erode(obstacle_mask, None, iterations=2)


    # Bitwise-AND mask and original image
    masked_ball = cv2.bitwise_and(frame, frame, mask=ball_mask)
    masked_obstacle = cv2.bitwise_and(frame, frame, mask=obstacle_mask)

    # detect edges using Canny detect
    edged_ball = cv2.Canny(masked_ball, 35, 125)
    edged_obstacle = cv2.Canny(masked_obstacle, 35, 125)

    # =================================================================
    DistanceB(edged_ball)
    BearingB(edged_ball)
    DistanceO(edged_obstacle)
    BearingO(edged_obstacle)
    # =================================================================

    # Calculate FPS
    endTime = time.time()
    timeTot = endTime - startTime
    # 1 frame / time taken to process
    if timeTot != 0.0:
        fps = 1 / timeTot
    print("FPS: %.2f" % fps)
    cv2.putText(frame, "FPS: %.2f" % fps,
                (frame.shape[1] - 320, frame.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX,
                0.5, (0, 255, 0), 2)

    # Display steps of masking images as video
    cv2.imshow('frame', frame)
    # cv2.imshow('blur', blur)
    # cv2.imshow('individual ball mask', ball_mask)
    # cv2.imshow('individual obstacle mask', obstacle_mask)
    # cv2.imshow('masked ball', masked_ball)
    # cv2.imshow('masked obstacle', masked_obstacle)
    # cv2.imshow('edged ball', edged_ball)
    # cv2.imshow('edged obstacle', edged_obstacle)

    k = cv2.waitKey(5) & 0xFF
    # k == 27 is for Esc key
    if k == 27:
        break
    # cv2.waitKey(0)  # Exit on keypress


cap.release()  # Release the camera object
cv2.destroyAllWindows()  # stops program
