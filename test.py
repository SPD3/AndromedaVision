import cv2
import numpy as np

cap = cv2.VideoCapture(0)
lowerLimit = cv2.scaleAdd(25,0,10)
upperLimit = cv2.scaleAdd(75,255,245)
erodedFrame = np.zeros((512,512,3), np.uint8)
params = cv2.SimpleBlobDetector_Params()
params.minDistBetweenBlobs=1000
params.minThreshold=1
params.maxThreshold=255
params.thresholdStep=1
params.minRepeatability = 0
params.filterByArea=True
params.minArea=2000
params.maxArea=100000
params.filterByCircularity=False
params.filterByColor=False
params.filterByConvexity=False
params.filterByInertia=False
params.filterByCircularity
detector = cv2.SimpleBlobDetector_create(params)


while True:
    ret, frame = cap.read()
    HSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    newFrame = cv2.inRange(HSV,(25,0,10),(75,255,245))
    erodedFrame = cv2.erode(newFrame,(300,300))
    GaussianBlurImage = cv2.GaussianBlur(newFrame,(3,3),1.6)
    erodedFrame = cv2.erode(GaussianBlurImage,(3,3))
    erodedFrame = cv2.erode(erodedFrame,(3,3))
    keypoints = detector.detect(erodedFrame)
    imWithKeypoint = cv2.drawKeypoints(erodedFrame, keypoints,
                                    np.array([]), (0,0,255),
                                    cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    
    cv2.imshow('image', imWithKeypoint)
    if cv2.waitKey(30)& 0xFF == ord('q'):
        break
cap.release()
cv2.destroyWindow('image')
