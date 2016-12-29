import numpy as np
import cv2

img = np.zeros((512,512,3), np.uint8)
cv2.rectangle(img, (10,10), (20,80),(0,255,0), -1)
cv2.rectangle(img, (10,70), (80,80),(0,255,0), -1)
cv2.rectangle(img, (70,10), (80,80),(0,255,0), -1)

cv2.rectangle(img, (10,160), (20,230),(0,255,0),-1)
cv2.rectangle(img, (10,160), (80,170),(0,255,0),-1)
cv2.rectangle(img, (70,160), (80,230),(0,255,0),-1)

cv2.rectangle(img, (160,10), (230,20),(0,255,0),-1)
cv2.rectangle(img, (160,10), (170,80),(0,255,0),-1)
cv2.rectangle(img, (160,70), (230,80),(0,255,0),-1)

cv2.imshow("UShapeImage",img)
cv2.waitKey()
#cv2.imwrite("C:\Users\Public\Pictures\Sample Pictures\UShapeImage5.png",img)
cv2.destroyAllWindows()
