import cv2
import numpy as np

cap = cv2.VideoCapture(0)

while True:
    
    ret, frame = cap.read()
    if ret == True:

        HSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        newFrame = cv2.inRange(HSV,(59,0,50),(61,255,200))
        newFrame = cv2.GaussianBlur(newFrame,(3,3),1.6)
        newFrame = cv2.erode(newFrame,(300,300))
       
        newFrame, contours, hierarchy = cv2.findContours(newFrame, cv2.RETR_LIST,cv2.CHAIN_APPROX_NONE)
        cv2.drawContours(newFrame,contours,-1,(255,255,255),5)

        cv2.imshow("Original", frame)
        cv2.imshow('image', newFrame)
        if cv2.waitKey(30)& 0xFF == ord('q'):
            break
    else:
        print "No Camera Connected!"
        break
cap.release()
cv2.destroyAllWindows()
