import cv2
import numpy as np

cap = cv2.VideoCapture(0)
x = 1

while(x == 1):
    ret, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    cv2.imshow('frame', gray)
    if cv2.waitKey(30)& 0xFF == ord('q'):
        break
cap.release()
cv2.destroyWindow('frame')
