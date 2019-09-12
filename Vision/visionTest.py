# import modules
import cv2
import numpy as np
# import picamera
# from picamera.array import PiArrayOutput
# from picamera import PiCamera

# Exposure off
# camera = PiCamera(sensor_mode=2)
# camera.exposure_mode = 'off'      # ... locks gains

# Auto white balance set value
# camera.awb_mode = 'off'

# Begin video capture
cap = cv2.VideoCapture(0)  # Connect to camera 0 (or the only camera)
cap.set(3, 320)  # Set the width to 320
cap.set(4, 240)  # Set the height to 240

while 1:
    # Check each individual frame
    ret, frame = cap.read()
    # blur frame
    blur = cv2.GaussianBlur(frame, (5, 5), 0)
    # apply canny edge detection to image
    # Canny(image, threshold1, threshold2
    edge = cv2.Canny(blur, 100, 200)
    # Convert BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

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

    # Threshold the HSV image
    ball_mask = cv2.inRange(hsv, lower_ball, upper_ball)
    blue_mask = cv2.inRange(hsv, lower_ball, upper_ball)
    yellow_mask = cv2.inRange(hsv, lower_ball, upper_ball)
    obstacle_mask = cv2.inRange(hsv, lower_ball, upper_ball)

    # Bitwise-AND mask and original image
    newFrame = cv2.bitwise_and(frame, frame, mask=ball_mask)
    newFrame = cv2.bitwise_and(frame, frame, mask=blue_mask)
    newFrame = cv2.bitwise_and(frame, frame, mask=yellow_mask)
    newFrame = cv2.bitwise_and(frame, frame, mask=obstacle_mask)

    # Display steps of masking images as video
    cv2.imshow('frame', frame)
    # cv2.imshow('blur', blur)
    cv2.imshow('edge', edge)
    # cv2.imshow('mask', ball_mask)
    cv2.imshow('new', newFrame)

    # ***********************************************************************



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
