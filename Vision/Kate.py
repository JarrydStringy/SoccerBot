# from picamera.array import PiRGBArray
# from picamera import PiCamera
import numpy as np
import cv2
import time
import math


class visionProcess():
    def __init__(self):
        # threading.Thread.__init__(self)
        # initialize the camera and grab a reference to the raw camera capture
        print("initialising")
        self.camera = cv2.VideoCapture(0)

        # allow the camera to warmup
        time.sleep(1)

        print("not failed")

        self.img = None
        self.hsv_img = None

        while (self.camera.isOpened()):
            ret, frame = self.camera.read()
            if ret == True:
                self.img = np.array(frame)
                self.hsv_img = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)
                print("great success")

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            else:
                break

        print("initialised")
        self.finish = False

        self.degrees_pixel = 0.07775
        self.focal_length = 591.07  # focal length of the camera in pixels
        self.fov = 62.2
        self.width_ball = 4.3  # width of the ball in cm
        self.width_obstacle = 18.0  # width of the obstacle in cm
        self.width_goal = 700.0  # width of the obstacle in cm

    def stop(self):
        self.finish = True
        self.camera.close()
        self.camera.release()
        cv2.destroyAllWindows()

    def refresh(self):
        while (self.camera.isOpened()):
            ret, frame = self.camera.read()
            if ret == True:
                self.img = np.array(frame)
                self.hsv_img = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)
                print("great success2")
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            else:
                break

    def allRB(self):
        # Finding Ball
        lower_orange = np.array([0, 25, 0])
        upper_orange = np.array([15, 255, 255])
        threshorange = cv2.inRange(self.hsv_img, lower_orange, upper_orange)
        threshorange = cv2.erode(threshorange, None, iterations=2)
        threshorange = cv2.dilate(threshorange, None, iterations=2)

        orange_cnts = cv2.findContours(threshorange.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

        if len(orange_cnts) == 0:
            ball_xpos = None
            ball_ypos = None
            ball_radius = None
            ball_bearing = None
            ball_distance = None

        elif len(orange_cnts) > 0:
            cnts = max(orange_cnts, key=cv2.contourArea)  # largest contour
            ((ball_xpos, ball_ypos), ball_radius) = cv2.minEnclosingCircle(cnts)  # circle

            M = cv2.moments(cnts)  # Centroid
            #        ball_centre = ( int(M['m10'] / M['m00']), int(M['m01'] / M['m00']) )

            #        cv2.circle(copy_img, (int(ball_xpos), int(ball_ypos)), int(ball_radius), (0, 255, 0), 2)
            #        cv2.circle(copy_img, ball_centre, 1, (0,0,255), -1)

            if ball_radius > 1:

                ballRB[0] = (self.focal_length * self.width_ball) / (2 * ball_radius)  # distance to object
                ballRB[1] = ((self.fov / 2) - self.degrees_pixel * (
                    math.sqrt((int(M["m10"] / M["m00"]) ** 2) + (int(M["m01"] / M["m00"]) ** 2))))  # bearing to object

                if cv2.contourArea(cnts) < 100:
                    ball_bearing = None
                    ball_distance = None

        #     #Finding Obstacles

        lower_black = np.array([0, 0, 0])
        upper_black = np.array([180, 110, 54])
        threshblack = cv2.inRange(hsv_img, lower_black, upper_black)
        threshblack = cv2.erode(threshblack, None, iterations=2)
        threshblack = cv2.dilate(threshblack, None, iterations=2)

        black_cnts = cv2.findContours(threshblack.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[-2]
        black_cnts = sorted(black_cnts, key=cv2.contourArea)
        black_cnts = black_cnts[0:3]

        if len(black_cnts) == 0:
            obstacle_width = None
            obstacle_height = None

        elif len(black_cnts) > 0:
            # loop over the contours
            for cnts in black_cnts:
                if cv2.contourArea(cnts) > 200:
                    (obstacle_xpos, obstacle_ypos, obstacle_width, obstacle_height) = cv2.boundingRect(cnts)

                    if obstacle_height > obstacle_width:
                        M = cv2.moments(cnts)
                        #                obstacle_center = (int(M['m10'] / M['m00']), int(M['m01'] / M['m00']))

                        #                cv2.rectangle(copy_img, (int(obstacle_xpos), int(obstacle_ypos)), (int(obstacle_xpos + obstacle_width), int(obstacle_ypos + obstacle_height)), (255, 0, 0), 2)

                        obstacleRB[0] = (self.focal_length * self.width_obstacle) / obstacle_width
                        obstacleRB[1] = ((self.fov / 2) - self.degrees_pixel * (
                            math.sqrt((int(M["m10"] / M["m00"]) ** 2) + (int(M["m01"] / M["m00"]) ** 2))))

        #     #Finding Yellow Goal

        lower_yellow = np.array([23, 170, 55])
        upper_yellow = np.array([74, 255, 255])
        threshyellow = cv2.inRange(hsv_img, lower_yellow, upper_yellow)
        threshyellow = cv2.erode(threshyellow, None, iterations=2)
        threshyellow = cv2.dilate(threshyellow, None, iterations=2)

        yellow_cnts = cv2.findContours(threshyellow.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[-2]
        yellow_cnts = sorted(yellow_cnts, key=cv2.contourArea)
        yellow_cnts = yellow_cnts[0:4]
        ygoalRB = None

        if len(yellow_cnts) == 0:
            ygoal_width = None
            ygoal_height = None

        elif len(yellow_cnts) > 0:
            # loop over the contours
            for cnts in yellow_cnts:
                if cv2.contourArea(cnts) > 500:
                    (ygoal_xpos, ygoal_ypos, ygoal_width, ygoal_height) = cv2.boundingRect(cnts)

                    M = cv2.moments(cnts)
                    #            ygoal_center = (int(M['m10'] / M['m00']), int(M['m01'] / M['m00']))

                    #            cv2.rectangle(copy_img, (int(ygoal_xpos), int(ygoal_ypos)), (int(ygoal_xpos + ygoal_width), int(ygoal_ypos + ygoal_height)), (0, 0, 255), 2)

                    ygoalRB[0] = (self.focal_length * self.width_goal) / ygoal_width  # distance to object
                    ygoalRB[1] = ((self.fov / 2) - self.degrees_pixel * (math.sqrt(
                        (int(M["m10"] / M["m00"]) ** 2) + (int(M["m01"] / M["m00"]) ** 2))))  # bearing to object

        # Finding Blue Goal
        lower_blue = np.array([76, 100, 30])
        upper_blue = np.array([218, 255, 255])
        threshblue = cv2.inRange(hsv_img, lower_blue, upper_blue)
        threshblue = cv2.erode(threshblue, None, iterations=2)
        threshblue = cv2.dilate(threshblue, None, iterations=2)

        blue_cnts = cv2.findContours(threshblue.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[-2]
        blue_cnts = sorted(blue_cnts, key=cv2.contourArea)
        blue_cnts = blue_cnts[0:4]
        bgoalRB = None

        if len(blue_cnts) == 0:
            bgoal_width = None
            bgoal_height = None


        elif len(blue_cnts) > 0:
            # loop over the contours
            for cnts in blue_cnts:
                if cv2.contourArea(cnts) > 500:
                    (bgoal_xpos, bgoal_ypos, bgoal_width, bgoal_height) = cv2.boundingRect(cnts)

                    M = cv2.moments(cnts)
                    #            bgoal_center = (int(M['m10'] / M['m00']), int(M['m01'] / M['m00']))

                    #            cv2.rectangle(copy_img, (int(bgoal_xpos), int(bgoal_ypos)), (int(bgoal_xpos + bgoal_width), int(bgoal_ypos + bgoal_height)), (0, 0, 255), 2)

                    bgoalRB[0] = (self.focal_length * self.width_goal) / bgoal_width  # distance to object
                    bgoalRB[1] = ((self.fov / 2) - self.degrees_pixel * (math.sqrt(
                        (int(M["m10"] / M["m00"]) ** 2) + (int(M["m01"] / M["m00"]) ** 2))))  # bearing to object

        return (ballRB, obstacleRB, ygoalRB, bgoalRB)

    def process(self):
        # time.sleep(0.01)
        print("ok")
        all_RB = self.allRB()
        return all_RB


if __name__ == '__main__':
    print("1")
    vision = visionProcess()
    while (1):
        print("2")
        vision.process()
        print("3")
        vision.refresh()
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
    vision.stop()
