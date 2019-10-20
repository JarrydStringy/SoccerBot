import math
import time
import numpy as np

import visTest as v


def gotoball(range, bearing):

    ball_connected = soccerBotSim.BallInDribbler()
    if ballRB[1] < 0.05 and ballRB[1] > -0.05:
        soccerBotSim.SetTargetVelocities(0.1, 0, 0)
    elif ballRB[1] > 0.05 and ballRB[0] > 0.2:
        soccerBotSim.SetTargetVelocities(0.1, 0, 0.2)
    elif ballRB[1] < -0.05 and ballRB[0] > 0.2:
        soccerBotSim.SetTargetVelocities(0.1, 0, -0.2)
    if range < 0.1:
        while ball_connected == False:
           soccerBotSim.SetTargetVelocities(0.1, 0, 0) 
           ball_connected = soccerBotSim.BallInDribbler()
           soccerBotSim.UpdateObjectPositions()

def goToGoal(range, bearing):
    if bearing < 0.05 and bearing > -0.05:
        soccerBotSim.SetTargetVelocities(0.1, 0, 0)
    elif bearing > 0.05 and range > 0.2:
        soccerBotSim.SetTargetVelocities(0.1, 0, 0.2)
    elif bearing < -0.05 and range > 0.2:
        soccerBotSim.SetTargetVelocities(0.1, 0, -0.2)
    
    if range < 0.6:
        soccerBotSim.SetTargetVelocities(0, 0, 0)
        soccerBotSim.KickBall(0.5)


if __name__ == '__main__':

	# Wrap everything in a try except case that catches KeyboardInterrupts. 
	# In the exception catch code attempt to Stop the VREP Simulator so don't have to Stop it manually when pressing CTRL+C
    try:

		# Create VREP SoccerBot object - this will attempt to open a connection to VREP. Make sure the VREP simulator is running.
        #soccerBotSim = VREP_SoccerBot('127.0.0.1', robotParameters, sceneParameters)
        #soccerBotSim.StartSimulator()
        in_view = False
        ball_behind_obstacle = False

        while True:
			# move the robot at a forward velocity of 0.1m/s with a rotational velocity of 0.5 rad/s.
			
			# Get Detected Objects
            ballRange, ballBearing = vision.BallReadings()

			# Check to see if the ball is within the camera's FOV
            if ballRB != None:
                print("Ball range and bearing: %0.4f, %0.4f"%(ballRange, ballBearing))
                gotoball(ballRange, ballBearing)
            elif ball_behind_obstacle == False and ball_connected == False:
                print("Search")
                soccerBotSim.SetTargetVelocities(0.001, 0, 0.8)
                if (blueRB != None or yellowRB != None) and in_view == False and rotation_count < 3:
                    rotation_count += 1
                    in_view = True
                elif blueRB == None and yellowRB == None and in_view == True and rotation_count < 3:
                    in_view = False 
                elif rotation_count == 3:
                    ball_behind_obstacle = True
            elif ball_behind_obstacle == True:
                soccerBotSim.SetTargetVelocities(0, 0, 0)
                print("Ball obstructed")

            if ball_connected == True and blueGoal == True:
                rotation_count = 0
                if blueRB != None:
                    goToGoal(blueRB[0],blueRB[1])
                else:
                    print("Searching for blue goal")
                    soccerBotSim.SetTargetVelocities(0.001,0,1)
            elif ball_connected == True and blueGoal == False:
                rotation_count = 0
                if yellowRB != None:
                    goToGoal(yellowRB[0],yellowRB[1])
                else:
                    print("Searching for yellow goal")
                    soccerBotSim.SetTargetVelocities(0.001,0,1)

			# Check to see if any obstacles are within the camera's FOV
            if obstaclesRB != None:
				# loop through each obstacle detected using Pythonian way
                for obstacle in obstaclesRB:
                    obstacleRange = obstacle[0]
                    obstacleBearing = obstacle[1]
    except KeyboardInterrupt as e: