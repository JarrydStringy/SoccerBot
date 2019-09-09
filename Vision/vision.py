# Imports
import cv2
import numpy as np

cap = cv2.VideoCapture(0)  # Connect to camera 0 (or the only camera)
cap.set(3, 320)  # Set the width to 320
cap.set(4, 240)  # Set the height to 240
ret, frame = cap.read()  # Get a frame from the camera

if ret:  # Check if data was obtained successfully
    cv2.imshow("CameraImage", frame)  # Display the obtained frame in a window called "CameraImage"
    cv2.waitKey(0)  # Make the program wait until you press a key before continuing.

    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  # Convert from BGR to HSV colour space)
    cv2.imshow("HSV_IMAGE", hsv_frame)

    h, s, v = cv2.split(frame)
    h *= 0
    v *= 0 

    hsv_frame = cv2.merge((h, s, v))
    cv2.imshow("HSV_IMAGE", hsv_frame)

    
    cv2.waitKey(0)  # Exit on keypress

cap.release()  # Release the camera object
cv2.destroyAllWindows()  # Close all opencv pop-up windows

