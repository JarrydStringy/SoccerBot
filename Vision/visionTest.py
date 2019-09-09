import cv2
import numpy as np

cap = cv2.VideoCapture(0)

while(1):
    # Take each frame
    _, frame = cap.read()

    # Convert BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    #Ball=============================================
    # define range in HSV
    #lower = np.array([250,122,91])
    #upper = np.array([251,250,255])
    lower_ball = np.array([0, 122, 91])
    upper_ball = np.array([255, 250, 255])     

    #Blue goal========================================
    lower_blue = np.array([115, 59, 34])
    upper_blue = np.array([152, 161, 136])

    #Yellow goal======================================
    lower_yellow = np.array([17, 178, 73])
    upper_yellow = np.array([30, 255, 193])

    #Obstacles========================================
    lower_obstacle = np.array([249, 53, 0])
    upper_obstacle = np.array([50, 160, 82])
    
    
    # Threshold the HSV image
    ball_mask = cv2.inRange(hsv, lower_ball, upper_ball)
    blue_mask = cv2.inRange(hsv, lower_ball, upper_ball)
    yellow_mask = cv2.inRange(hsv, lower_ball, upper_ball)
    obstacle_mask = cv2.inRange(hsv, lower_ball, upper_ball)

    # Bitwise-AND mask and original image
    new = cv2.bitwise_and(frame, frame, mask= ball_mask)
    new = cv2.bitwise_and(frame, frame, mask= blue_mask)
    new = cv2.bitwise_and(frame, frame, mask= yellow_mask)
    new = cv2.bitwise_and(frame, frame, mask= obstacle_mask)

    cv2.imshow('frame',frame)
    #cv2.imshow('mask',ball_mask)
    cv2.imshow('new',new)
    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break

cv2.destroyAllWindows()
