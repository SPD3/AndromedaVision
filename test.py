import cv2
import numpy as np

cap = cv2.VideoCapture(0)
x = 1
lowerLimit = cv2.scaleAdd(25,0,10)
upperLimit = cv2.scaleAdd(75,255,245)
while(x == 1):
    ret, frame = cap.read()
    HSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    newFrame = cv2.inRange(HSV,(25,0,10),(75,255,245))
    cv2.imshow('frame', newFrame)
    if cv2.waitKey(30)& 0xFF == ord('q'):
        break
cap.release()
cv2.destroyWindow('frame')
