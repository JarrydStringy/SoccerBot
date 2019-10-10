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


def BoxObject(marker):
    # Draw a box around objects
    box = cv2.cv.BoxPoints(marker) if imutils.is_cv2() else cv2.boxPoints(marker)
    box = np.int0(box)
    cv2.drawContours(frame, [box], -1, (0, 255, 0), 2)


def PrintReadings(dist, angle, yd, yb, name):
    # Print readings on frame
    cv2.putText(frame, "D %s: %.2fm" % (name, dist),
                (frame.shape[1] - 110, frame.shape[0] - yd), cv2.FONT_HERSHEY_SIMPLEX,
                0.45, (0, 255, 0), 2)
    cv2.putText(frame, "B %s: %.2fdeg" % (name, angle),
                (frame.shape[1] - 110, frame.shape[0] - yb), cv2.FONT_HERSHEY_SIMPLEX,
                0.45, (0, 255, 0), 2)


def BallReadings(image):
    # find the contours in the edged image and keep the largest one
    contours = cv2.findContours(image.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    contours = imutils.grab_contours(contours)
    if len(contours) > 0:
        c = max(contours, key=cv2.contourArea)
        marker = cv2.minAreaRect(c)
        BoxObject(marker)
        # Find the distances and bearings
        dist_ball = DistanceToCamera(BALL_WIDTH, FOCAL_LENGTH, marker[1][0])
        angle_ball = BearingToCamera(marker[1][0], DEG_PER_PX)
        # Print
        print("Distance to Ball: " + str(dist_ball))
        print("Angle to Ball: " + str(angle_ball))
        PrintReadings(dist_ball, angle_ball, 70, 50, "ball")
        ballRB = [dist_ball, angle_ball]
        return ballRB


def ObstacleReadings(image):
    # find the contours in the edged image and keep the largest one
    contours = cv2.findContours(image.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    contours = imutils.grab_contours(contours)
    if len(contours) > 0:
        c = max(contours, key=cv2.contourArea)
        marker = cv2.minAreaRect(c)
        BoxObject(marker)
        # Find the distances and bearings
        dist_obstacle = DistanceToCamera(OBSTACLE_WIDTH, FOCAL_LENGTH, marker[1][0])
        angle_obstacle = BearingToCamera(marker[1][0], DEG_PER_PX)
        # Print
        print("Distance to Obstacle: " + str(dist_obstacle))
        print("Angle to Obstacle: " + str(angle_obstacle))
        PrintReadings(dist_obstacle, angle_obstacle, 30, 10, "obs")
        obstacleRB = [dist_obstacle, angle_obstacle]
        return obstacleRB


def BlueGoalReadings(image):
    # find the contours in the edged image and keep the largest one
    contours = cv2.findContours(image.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    contours = imutils.grab_contours(contours)
    if len(contours) > 0:
        c = max(contours, key=cv2.contourArea)
        marker = cv2.minAreaRect(c)
        BoxObject(marker)
        # Find the distances and bearings
        dist_blue = DistanceToCamera(OBSTACLE_WIDTH, FOCAL_LENGTH, marker[1][0])
        angle_blue = BearingToCamera(marker[1][0], DEG_PER_PX)
        # Print
        print("Distance to Blue Goal: " + str(dist_blue))
        print("Angle to Blue Goal: " + str(angle_blue))
        PrintReadings(dist_blue, angle_blue, 150, 130, "blue")
        blueRB = [dist_blue, angle_blue]
        return blueRB


def YellowGoalReadings(image):
    # find the contours in the edged image and keep the largest one
    contours = cv2.findContours(image.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    contours = imutils.grab_contours(contours)
    if len(contours) > 0:
        c = max(contours, key=cv2.contourArea)
        marker = cv2.minAreaRect(c)
        BoxObject(marker)
        # Find the distances and bearings
        dist_yellow = DistanceToCamera(YELLOW_GOAL_WIDTH, FOCAL_LENGTH, marker[1][0])
        angle_yellow = BearingToCamera(marker[1][0], DEG_PER_PX)
        # Print
        print("Distance to Obstacle: " + str(dist_yellow))
        print("Angle to Obstacle: " + str(angle_yellow))
        PrintReadings(dist_yellow, angle_yellow, 110, 90, "ylw")
        yellowRB = [dist_yellow, angle_yellow]
        return yellowRB


def WallReadings(image):
    # find the contours in the edged image and keep the largest one
    contours = cv2.findContours(image.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    contours = imutils.grab_contours(contours)
    if len(contours) > 0:
        c = max(contours, key=cv2.contourArea)
        marker = cv2.minAreaRect(c)
        BoxObject(marker)
        # Find the distances and bearings
        dist_wall = DistanceToCamera(WALL_WIDTH, FOCAL_LENGTH, marker[1][0])
        angle_wall = BearingToCamera(marker[1][0], DEG_PER_PX)
        # Print
        print("Distance to Wall: " + str(dist_wall))
        print("Angle to Wall: " + str(angle_wall))
        PrintReadings(dist_wall, angle_wall, 190, 170, "wall")


def Readings():
    return BallReadings(frame)


# Filter each image to find required objects====================================================
# Record starting time for FPS
startTime = time.time()

ret, frame = cap.read()  # capture each frame
# frame = cv2.imread('distTest60.png')
frame = cv2.flip(frame, 0)  # Flip image
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

# Calculate FPS================================================================================
endTime = time.time()
timeTot = endTime - startTime
# 1 frame / time taken to process
if timeTot != 0.0:
    fps = 1 / timeTot
print("FPS: %.2f" % fps)
cv2.putText(frame, "FPS: %.2f" % fps,
            (frame.shape[1] - 320, frame.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX,
            0.5, (0, 255, 0), 2)

# Display steps of masking images as video======================================================
cv2.imshow('frame', frame)
# cv2.imshow('blur', blur)
# cv2.imshow('individual ball mask', ball_mask)
# cv2.imshow('masked ball', masked_wall)
# cv2.imshow('edged ball', edged_obstacle)


cap.release()  # Release the camera object
cv2.destroyAllWindows()  # stops program
