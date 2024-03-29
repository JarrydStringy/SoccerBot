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
# camera.awb_mode = 'off'

# define ranges of spectrum in HSV[] for each object======================================================
# Ball=============================================
# lower_ball = np.array([0, 122, 91])
# upper_ball = np.array([255, 250, 255])
# lower_ball = np.array([0, 135, 150])
# upper_ball = np.array([255, 250, 255])

# lower_ball = np.array([0, 0, 140])
# upper_ball = np.array([16, 250, 255])

lower_ball = np.array([0, 0, 140])
upper_ball = np.array([16, 250, 255])
# Blue goal========================================
lower_blue = np.array([115, 59, 34])
upper_blue = np.array([152, 161, 136])
# Yellow goal======================================
lower_yellow = np.array([17, 178, 73])
upper_yellow = np.array([30, 255, 193])
# Obstacles========================================
lower_obstacle = np.array([249, 53, 0])
upper_obstacle = np.array([50, 160, 82])
# Walls============================================
lower_wall = np.array([249, 53, 0])
upper_wall = np.array([50, 160, 82])

BALL_WIDTH = 0.043  # m
OBSTACLE_WIDTH = 0.18  # m
FOCAL_LENGTH = 123.423  # m
DEG_PER_PX = 0.0192  # Calculated constant

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


# Distance to Ball
def DistanceB(image):
    # find the contours in the edged image and keep the largest one
    contours = cv2.findContours(image.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    contours = imutils.grab_contours(contours)
    if len(contours) > 0:
        c = max(contours, key=cv2.contourArea)

        marker = cv2.minAreaRect(c)

        dist = DistanceToCamera(BALL_WIDTH, FOCAL_LENGTH, marker[1][0])
        print("Distance to Ball: " + str(dist_ball))

        box = cv2.cv.BoxPoints(marker) if imutils.is_cv2() else cv2.boxPoints(marker)
        box = np.int0(box)
        cv2.drawContours(frame, [box], -1, (0, 255, 0), 2)
        cv2.putText(frame, "b: %.2fm" % dist_ball,
                    (frame.shape[1] - 100, frame.shape[0] - 100), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 0), 2)

        angle_obstacle = BearingToCamera(marker[1][0], DEG_PER_PX)
        print("Angle to Ball: " + str(angle_obstacle))
        cv2.putText(frame, "b: %.2fdeg" % angle_obstacle,
                    (frame.shape[1] - 100, frame.shape[0] - 70), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 0), 2)



# Distance to Obstacle
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
    # Filter each image to find required objects====================================================
    # Record starting time for FPS
    startTime = time.time()

    ret, frame = cap.read()     # capture each frame
    # frame = cv2.imread('Test_colour2.png')
    # frame = cv2.imread('test.png')
    frame = cv2.imread('distTest60.png')
    frame = cv2.flip(frame, 0)    # Flip image
    blur = cv2.GaussianBlur(frame, (5, 5), 0)       # blur frames with gaussian 5x5 kernel and determined std dev
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)     # Convert BGR to HSV

    # Threshold the HSV image and erode to create clear mask
    ball_mask = cv2.inRange(hsv, lower_ball, upper_ball)
    ball_mask = cv2.erode(ball_mask, None, iterations=2)

    obstacle_mask = cv2.inRange(hsv, lower_obstacle, upper_obstacle)
    obstacle_mask = cv2.erode(obstacle_mask, None, iterations=2)

    blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)
    blue_mask = cv2.erode(blue_mask, None, iterations=2)

    yellow_mask = cv2.inRange(hsv, lower_obstacle, upper_yellow)
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
    DistanceB(edged_ball)
    BearingB(edged_ball)
    DistanceO(edged_obstacle)
    BearingO(edged_obstacle)

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
