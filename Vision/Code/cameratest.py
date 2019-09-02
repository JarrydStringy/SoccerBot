import cv2
import numpy as np

cap = cv2.VideoCapture(0)
cap.set(3, 320)
cap.set(4, 240)
ret, frame = cap.read() 

if ret == True:
    cv2.imshow("CameraImage", frame)
    
    hsv_frame = cv2.cvtColor(frame, cv2.Color_BGR2HSV)
    
    cv2.imshow("HSV_IMAGE", hsv_frame)
    cv2. waitKey(0)

cap.release()
cv2.destroyAllWindows()
