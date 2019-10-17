import visTest as v
while True:
    ballRB = v.BallReadings(v.edged_ball)
    ballRange = ballRB[0]
    ballBearing = ballRB[1]
    print("Ball Range: " + str(ballRange))
    print("Ball Bearing: " + str(ballBearing))
