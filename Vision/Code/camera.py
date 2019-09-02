#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Aug  5 14:19:39 2019

@author: pi
"""

import cv2
import numpy as np

cap = cv2.VideoCapture(0)
cap.set (3, 320)
cap.set(4, 240)
ret, frame = cap.read() 

if ret == True:
    cv2.imshow("CameraImage", frame)
    
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    cv2.imshow("HSV_IMAGE", hsv_frame)                                
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_HSV2GRAY)
    
    ret, thresholded_frame = cv2.threshold(gray_frame, 127, 255, cv2.THRESH_BINARY)
    cv2.imshow("Thresholded". thresholded_frame)
    cv2.waitkey(0)

cap.release()
cv2.destroyAllWindows()
