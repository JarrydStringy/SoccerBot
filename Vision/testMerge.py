import visTest as v
import cv2
old = 0

while True:
    ballRB = v.BallReadings(v.edged_ball)
    ballRange = ballRB[0]
    ballBearing = ballRB[1]
    if ballRange != old or ballRange != 0:
        old = ballRange
        print("Ball Range: " + str(ballRange))      # 0.64
        print("Ball Bearing: " + str(ballBearing))  # 0.16
        print("in")
    cv2.imshow('edged ball', v.edged_ball)

    v.Display(v.frame)
