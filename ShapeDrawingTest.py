import numpy as np
import cv2

img = np.zeros((512,512,3), np.uint8)
cv2.rectangle(img,(0,0),(512/7,512),(0,255,0),-1)
cv2.rectangle(img,(0,512-512/7),(512,512),(0,255,0),-1)
cv2.rectangle(img,(512 - 512/7,0),(512,512),(0,255,0),-1)
cv2.rectangle(img,(100,0),(101,7),(0,255,0),-1)
cv2.rectangle(img,(100,6),(107,7),(0,255,0),-1)
cv2.rectangle(img,(106,0),(107,7),(0,255,0),-1)
cv2.rectangle(img,(250,250),(250 + 50/7,300),(0,255,0),-1)
cv2.rectangle(img,(250,300-50/7),(300,300),(0,255,0),-1)
cv2.rectangle(img,(300-50/7,250),(300,300),(0,255,0),-1)

cv2.rectangle(img,(350,50),(400,100),(0,255,0),-1)

cv2.imshow("UShapeImage",img)
cv2.waitKey()
cv2.imwrite("C:\Users\Public\Pictures\Sample Pictures\UShapeImage6.png",img)
cv2.destroyAllWindows()
