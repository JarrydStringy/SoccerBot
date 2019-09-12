# import modules
import cv2
import numpy as np
import imutils
from collections import deque
import argparse
# import picamera
# from picamera.array import PiArrayOutput
# from picamera import PiCamera

# Exposure off
# camera = PiCamera(sensor_mode=2)
# camera.exposure_mode = 'off'      # ... locks gains

# Auto white balance set value
# camera.awb_mode = 'off'

# define ranges of spectrum in HSV[] for each object
# Ball=============================================
lower_ball = np.array([0, 122, 91])
upper_ball = np.array([255, 250, 255])
# Blue goal========================================
lower_blue = np.array([115, 59, 34])
upper_blue = np.array([152, 161, 136])
# Yellow goal======================================
lower_yellow = np.array([17, 178, 73])
upper_yellow = np.array([30, 255, 193])
# Obstacles========================================
lower_obstacle = np.array([249, 53, 0])
upper_obstacle = np.array([50, 160, 82])

pts = deque()

ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video", help="path to the (optional) video file")
ap.add_argument("-b", "--buffer", type=int, default=64,
                help="max buffer size")
args = vars(ap.parse_args())

# Begin video capture
cap = cv2.VideoCapture(0)  # Connect to camera 0 (or the only camera)
cap.set(3, 320)  # Set the width to 320
cap.set(4, 240)  # Set the height to 240

while True:
    # capture each frame
    check, frame = cap.read()
    # blur frames with gaussian 5x5 kernel and determined std dev
    blur = cv2.GaussianBlur(frame, (5, 5), 0)
    # Convert BGR to HSV
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

    # Threshold the HSV image
    ball_mask = cv2.inRange(hsv, lower_ball, upper_ball)                    # can erode and dilate
    # blue_mask = cv2.inRange(hsv, lower_ball, upper_ball)
    # yellow_mask = cv2.inRange(hsv, lower_ball, upper_ball)
    # obstacle_mask = cv2.inRange(hsv, lower_ball, upper_ball)

    cnts = cv2.findContours(ball_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    centre = None

    if len(cnts) > 0:
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        centre = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

        if radius > 10:
            cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
            cv2.circle(frame, centre, 5, (0, 0, 255), -1)

    pts.appendleft(centre)
    for i in range(1, len(pts)):
        if pts[i -1] is None or pts[i] is None:
            continue
        thickness = int(np.sqrt((args["buffer"] / float(i + 1)) * 2.5))
        cv2.line(frame, pts[i -1], pts[i], (0, 0, 255), thickness)

    # Bitwise-AND mask and original image
    # masked = cv2.bitwise_and(frame, frame, mask=ball_mask)
    # masked = cv2.bitwise_and(frame, frame, mask=blue_mask)
    # masked = cv2.bitwise_and(frame, frame, mask=yellow_mask)
    # masked = cv2.bitwise_and(frame, frame, mask=obstacle_mask)

    # apply canny edge detection to image
    # edged = cv2.Canny(ball_mask, 100, 200)

    # Display steps of masking images as video
    cv2.imshow('frame', frame)
    # cv2.imshow('blur', blur)
    # cv2.imshow('individual mask', ball_mask)
    # cv2.imshow('masked', masked)
    # cv2.imshow('edged', edged)

    # ***********************************************************************

    # find the contours in the edged image and keep the largest one
    # contours = cv2.findContours(edge.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    # contours = imutils.grab_contours(contours)  # detects edges and grabs shape
    # c = max(contours, key=cv2.contourArea)  # finds the largest contour
    # print(c)
    # ***********************************************************************

    # & 0xFF sets left bits to 0 for keyboard input whilst waiting for input
    k = cv2.waitKey(5) & 0xFF
    # k == 27 is for Esc key
    if k == 27:
        break

    # cv2.waitKey(0)  # Exit on keypress

cap.release()  # Release the camera object
# stops program
cv2.destroyAllWindows()
