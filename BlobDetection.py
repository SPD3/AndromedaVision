import cv2
import numpy as np

#params = cv2.SimpleBlobDetector_Params()
#params.minDistBetweenBlobs=1000
#params.minThreshold=1
#params.maxThreshold=255
#params.thresholdStep=1
#print params.minThreshold, params.maxThreshold, params.thresholdStep
#print params.minRepeatability
#params.minRepeatability = 0
#params.filterByArea=True
#params.minArea=2000
#params.maxArea=100000
#params.filterByCircularity=False
#params.filterByColor=False
#params.filterByConvexity=False
#params.filterByInertia=False
#params.filterByCircularity
#detector = cv2.SimpleBlobDetector_create(params)
img = cv2.imread("C:\Users\Public\Pictures\Sample Pictures\OriginalColors.jpg")
HSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

cv2.imshow("original", img)
newFrame = cv2.inRange(HSV,(25,0,10),(75,255,245))
GaussianBlurImage = cv2.GaussianBlur(newFrame,(3,3),1.6)
erodedFrame = cv2.erode(GaussianBlurImage,(3,3))
erodedFrame = cv2.erode(erodedFrame,(3,3))
#keypoints = detector.detect(erodedFrame)
#x = len(keypoints)
#print x
#print keypoints

#imWithKeypoint = cv2.drawKeypoints(erodedFrame, keypoints,
                                    #np.array([]), (0,0,255),
                                    #cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
newFrame, contours, hierarchy = cv2.findContours(erodedFrame, cv2.RETR_LIST,cv2.CHAIN_APPROX_NONE)
cv2.drawContours(newFrame,contours,-1,(255,255,255))

cv2.imshow("ContourFrame", newFrame)
#cv2.imshow("ErodedImage", imWithKeypoint)
cv2.waitKey()
cv2.destroyAllWindows()
