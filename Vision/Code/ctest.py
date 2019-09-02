import numpy as np
import cv2

cap = cv2.VideoCapture(0)							
cap.set(3, 640)									
cap.set(4, 480)									

ret, frame = cap.read()								
frame_blue = frame[:,:,0];							# Extract blue channel
ret, thresholded_frame = cv2.threshold(frame_blue, 127, 255, cv2.THRESH_BINARY)	# Threshold blue channel

cv2.imshow("Binary Thresholded Frame", thresholded_frame)			# Display thresholded frame
cv2.waitKey(0)									# Exit on keypress

cap.release()
cv2.destroyAllWindows()
