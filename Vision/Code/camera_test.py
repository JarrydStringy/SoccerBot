import cv2
import numpy as np

cap = cv2.VideoCapture(0)
cap.set (3, 320)
cap.set(4, 240)
ret, frame = cap.read() 

if ret == True:
    cv2.imshow("CameraImage", frame)
    cv2. waitKey(0)
    
cv2.imwrite("frame0001.png", frame)

cap.release()
cv2.destroyAllWindows()
