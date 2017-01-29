import cv2
import numpy as np
import os
import math
# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((6*8,3), np.float32)
objp[:,:2] = np.mgrid[0:8,0:6].T.reshape(-1,2)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

images = "C:\\Users\\admin\\Pictures\\ChessBoardImages"
for imgFileName in os.listdir(images):    
    
    fullFileName = os.path.join(images, imgFileName)
    img = cv2.imread(fullFileName)

    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    h,w = gray.shape[:2]
    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, (6,8),None)

    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)

        corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
        imgpoints.append(corners2)

        # Draw and display the corners
        img = cv2.drawChessboardCorners(img, (6,8), corners2,ret)
        cv2.imshow('window',img)
        cv2.waitKey(0)
    
    cv2.destroyAllWindows()

ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, (w,h),None,None)

#np.savez(('C:\\Users\\admin\\OpenCV Experiments\\CameraCalibrationData'), mtx=mtx, dist=dist)

#np.savez(('/image_object_points'),objpoints=objpoints, imgpoints=imgpoints)


