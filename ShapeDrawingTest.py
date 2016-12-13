import numpy as np
import cv2

img = np.zeros((512,512,3), np.uint8)
cv2.line(img,(0,0),(0,511),(255,0,0),511)
cv2.imshow('Image', img)
k = cv2.waitKey(0)
if k == 27:
    cv2.destroyWindow('Image')
