# import modules
import cv2
import numpy as np
import imutils
import time

# Raspberry Pi camera manipulation=========================================================================
# Exposure off
# camera = PiCamera(sensor_mode=2)
# camera.exposure_mode = 'off'      # ... locks gains

# Auto white balance set value
# camera.awb_mode = 'sunlight'

# define ranges of spectrum in HSV[] for each object======================================================
# Ball=============================================
lower_ball = np.array([0, 0, 0])
upper_ball = np.array([13, 255, 255])
# Blue goal========================================
lower_blue = np.array([49, 50, 0])
upper_blue = np.array([100, 255, 125])
# Yellow goal======================================
lower_yellow = np.array([13, 100, 0])
upper_yellow = np.array([20, 255, 255])
# Obstacles========================================
lower_obstacle = np.array([0, 0, 0])
upper_obstacle = np.array([40, 255, 80])
# Walls============================================
lower_wall = np.array([0, 0, 0])
upper_wall = np.array([255, 60, 255])

# Constants
BALL_WIDTH = 0.043  # m
OBSTACLE_WIDTH = 0.18  # m
YELLOW_GOAL_WIDTH = 0.75  # m
BLUE_GOAL_WIDTH = 0.75  # m
WALL_WIDTH = 0  # m
FOCAL_LENGTH = 123.423  # m
DEG_PER_PX = 0.01927  # Calculated constant

# Begin video capture
cap = cv2.VideoCapture(0)  # Connect to camera 0 (or the only camera)
cap.set(3, 320)  # Set the width to 320
cap.set(4, 240)  # Set the height to 240


def DistanceToCamera(knownWidth, focalLength, perWidth):
    # compute and return the distance from the marker to the camera
    if perWidth != 0:
        return (knownWidth * focalLength) / perWidth
    else:
        return 0


def BearingToCamera(position, degreePixelRatio):
    # compute and return the distance from the maker to the camera
    return position * degreePixelRatio


def BallReadings(image):
    # find the contours in the edged image and keep the largest one
    contours = cv2.findContours(image.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    contours = imutils.grab_contours(contours)
    if len(contours) > 0:
        c = max(contours, key=cv2.contourArea)
        marker = cv2.minAreaRect(c)
        # Find the distances and bearings
        dist_ball = DistanceToCamera(BALL_WIDTH, FOCAL_LENGTH, marker[1][0])
        angle_ball = BearingToCamera(marker[1][0], DEG_PER_PX)
        ballRB = [dist_ball, angle_ball]
        return ballRB


def ObstacleReadings(image):
    # find the contours in the edged image and keep the largest one
    contours = cv2.findContours(image.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    contours = imutils.grab_contours(contours)
    if len(contours) > 0:
        c = max(contours, key=cv2.contourArea)
        marker = cv2.minAreaRect(c)
        # Find the distances and bearings
        dist_obstacle = DistanceToCamera(OBSTACLE_WIDTH, FOCAL_LENGTH, marker[1][0])
        angle_obstacle = BearingToCamera(marker[1][0], DEG_PER_PX)
        obstacleRB = [dist_obstacle, angle_obstacle]
        return obstacleRB


def BlueGoalReadings(image):
    # find the contours in the edged image and keep the largest one
    contours = cv2.findContours(image.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    contours = imutils.grab_contours(contours)
    if len(contours) > 0:
        c = max(contours, key=cv2.contourArea)
        marker = cv2.minAreaRect(c)
        # Find the distances and bearings
        dist_blue = DistanceToCamera(OBSTACLE_WIDTH, FOCAL_LENGTH, marker[1][0])
        angle_blue = BearingToCamera(marker[1][0], DEG_PER_PX)
        blueRB = [dist_blue, angle_blue]
        return blueRB


def YellowGoalReadings(image):
    # find the contours in the edged image and keep the largest one
    contours = cv2.findContours(image.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    contours = imutils.grab_contours(contours)
    if len(contours) > 0:
        c = max(contours, key=cv2.contourArea)
        marker = cv2.minAreaRect(c)
        # Find the distances and bearings
        dist_yellow = DistanceToCamera(YELLOW_GOAL_WIDTH, FOCAL_LENGTH, marker[1][0])
        angle_yellow = BearingToCamera(marker[1][0], DEG_PER_PX)
        yellowRB = [dist_yellow, angle_yellow]
        return yellowRB


def WallReadings(image):
    # find the contours in the edged image and keep the largest one
    contours = cv2.findContours(image.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    contours = imutils.grab_contours(contours)
    if len(contours) > 0:
        c = max(contours, key=cv2.contourArea)
        marker = cv2.minAreaRect(c)
        # Find the distances and bearings
        dist_wall = DistanceToCamera(WALL_WIDTH, FOCAL_LENGTH, marker[1][0])
        angle_wall = BearingToCamera(marker[1][0], DEG_PER_PX)
        wallRB = [dist_wall, angle_wall]
        return wallRB


while True:
    ret, frame = cap.read()  # capture each frame
    frame = cv2.flip(frame, 0)  # Flip image vertically
    frame = cv2.flip(frame, 1)  # Flip image horizontally
    blur = cv2.GaussianBlur(frame, (5, 5), 0)  # blur frames with gaussian 5x5 kernel and determined std dev
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)  # Convert BGR to HSV

    # Threshold the HSV image and erode to create clear mask
    ball_mask = cv2.inRange(hsv, lower_ball, upper_ball)
    ball_mask = cv2.erode(ball_mask, None, iterations=2)

    obstacle_mask = cv2.inRange(hsv, lower_obstacle, upper_obstacle)
    obstacle_mask = cv2.erode(obstacle_mask, None, iterations=2)

    blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)
    blue_mask = cv2.erode(blue_mask, None, iterations=2)

    yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    yellow_mask = cv2.erode(yellow_mask, None, iterations=2)

    wall_mask = cv2.inRange(hsv, lower_wall, upper_wall)
    wall_mask = cv2.erode(wall_mask, None, iterations=2)

    # Bitwise-AND mask and original image
    masked_ball = cv2.bitwise_and(frame, frame, mask=ball_mask)
    masked_obstacle = cv2.bitwise_and(frame, frame, mask=obstacle_mask)
    masked_blue = cv2.bitwise_and(frame, frame, mask=blue_mask)
    masked_yellow = cv2.bitwise_and(frame, frame, mask=yellow_mask)
    masked_wall = cv2.bitwise_and(frame, frame, mask=wall_mask)

    # detect edges using Canny detect
    edged_ball = cv2.Canny(masked_ball, 35, 125)
    edged_obstacle = cv2.Canny(masked_obstacle, 35, 125)
    edged_blue = cv2.Canny(masked_blue, 35, 125)
    edged_yellow = cv2.Canny(masked_yellow, 35, 125)
    edged_wall = cv2.Canny(masked_wall, 35, 125)

    # Return distance and bearing to all of the objects=============================================
    BallReadings(edged_ball)
    ObstacleReadings(edged_obstacle)
    YellowGoalReadings(edged_yellow)
    BlueGoalReadings(edged_blue)
    WallReadings(edged_wall)

cap.release()  # Release the camera object
cv2.destroyAllWindows()  # stops program
