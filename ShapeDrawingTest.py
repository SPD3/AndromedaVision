import numpy as np
import cv2

img = np.zeros((512,512,3), np.uint8)
def drawCircle(event,x,y,flags,param):
    if event == cv2.EVENT_LBUTTONDBLCLK:
        cv2.circle(img,(x,y),100,(255,255,255),10)

cv2.namedWindow('Image')
cv2.setMouseCallback('Image',drawCircle)
while(1):
    cv2.imshow('Image',img)
    if cv2.waitKey(20) == 27:
        break
cv2.destroyWindow('Image')
