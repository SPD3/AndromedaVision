import cv2
import numpy as np
import os
import math
# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((9*6,3), np.float32)
objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

images = "/home/pi/Pictures/IntrensicCameraCalibrationImages"
for imgFileName in os.listdir(images):    
    print "Processing ", imgFileName
    if imgFileName[-4:] != ".png":
        print "skipping..."
        continue
    fullFileName = os.path.join(images, imgFileName)
    img = cv2.imread(fullFileName)

    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    h,w = gray.shape[:2]
    # Find the chess board corners
    #ret, corners = cv2.findCirclesGrid(gray, (4, 11), None, cv2.CALIB_CB_ASYMMETRIC_GRID)
    ret, corners = cv2.findChessboardCorners(gray, (9, 6))
    
    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)
        print corners
        corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
        imgpoints.append(corners2)

        # Draw and display the corners
        #img = cv2.drawChessboardCorners(img, (6,8), corners2,ret)
        #small = cv2.resize(img, (0,0), fx = 0.3, fy = 0.3)
        #cv2.imshow('window',img)
        #cv2.waitKey(0)
    
    cv2.destroyAllWindows()
    break

print "calibrating camera..."
print 'objpoints', objpoints
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, (w,h),None,None)
print 'mtx is: ', mtx
print 'dist is ', dist
meanError = 0
for i in xrange(len(objpoints)):
    imagePoints2, none = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
    error = cv2.norm(imgpoints[i], imagePoints2, cv2.NORM_L2)/len(imagePoints2)
    meanError += error
print "Total Error: ", meanError/len(objpoints)

np.save(('/home/pi/Desktop/mtx.npy'), mtx)
np.save(('/home/pi/Desktop/dist.npy'), dist)


