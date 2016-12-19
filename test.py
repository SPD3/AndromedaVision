import cv2
import numpy as np

cap = cv2.VideoCapture(0)
lowerLimit = cv2.scaleAdd(25,0,10)
upperLimit = cv2.scaleAdd(75,255,245)
erodedFrame = np.zeros((512,512,3), np.uint8)
    
while True:
    ret, frame = cap.read()
    HSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    newFrame = cv2.inRange(HSV,(25,0,10),(75,255,245))
    erodedFrame = cv2.erode(newFrame,(300,300))
    cv2.imshow('image', newFrame)
    cv2.imshow('frame', erodedFrame)
    if cv2.waitKey(5000)& 0xFF == ord('q'):
        break
cap.release()
cv2.destroyWindow('frame')
cv2.destroyWindow('image')
