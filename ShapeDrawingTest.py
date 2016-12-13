import numpy as np
import cv2

img = np.zeros((512,512,3), np.uint8)
font = cv2.FONT_HERSHEY_SIMPLEX
cv2.line(img,(0,0),(0,511),(255,0,0),511)
cv2.rectangle(img,(10,10),(500,500),(0,255,0),-1)
cv2.circle(img,(255,255),30,(0,0,255),-1)
cv2.putText(img,'OpenCV',(20,450),font,4,(255,255,255),20,cv2.LINE_AA)
cv2.imshow('Image', img)
cv2.waitKey(0)
cv2.destroyWindow('Image')
