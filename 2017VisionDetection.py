#import pkg_resources
##print pkg_resources.get_distribution('picamera').version

import cv2
import numpy as np
import os
import math
from picamera.array import PiRGBArray
import picamera
import time
import sys
from networktables import NetworkTables
import logging


#interensic paramaters
m_xResolution = 2656 
m_yResolution = 1328
#m_cameraCalibrationData = np.load('/home/pi/test/AndromedaVision/CameraCalibrationData.npz')
m_cameraMatrix = np.matrix([[  1.69714761e+04,   0.00000000e+00,   1.20224572e+03],
 [  0.00000000e+00,   1.75260785e+04,   6.19148918e+02],
 [  0.00000000e+00,   0.00000000e+00,   1.00000000e+00]])
m_distCoeffs = np.matrix([[  3.12035247e+01,  -1.43758274e+03,   1.93277402e-01,  -3.79253300e-01,
   -6.08747037e+04]])
m_centerXOfImage = m_cameraMatrix[0,2]#m_xResolution/2# #Need to load in actual Numbers from Camera Calibration
m_centerYOfImage = m_cameraMatrix[1,2]# m_yResolution/2# #Need to load in actual Numbers from Camera Calibration
m_focalLengthOfCameraX = m_cameraMatrix[0,0] #Need to load in actual Numbers from Camera Calibration
m_focalLengthOfCameraY = m_cameraMatrix[1,1] #Need to load in actual Numbers from Camera Calibration
#m_horizonLine = 0.9 * m_yResolution # #Need to get actual number from camera

#field parameters
m_heightOfHighGoalTarget = 10.0 #Need to get actual number from manual
m_heightOfLiftTarget = 15.75 #Actual Number From manual
m_widthOfLift = 8.25 #Actual number from manual; Top Left corner of retroReflective to Top right Corner Of RetroReflective
m_widthOfRetroReflectiveToLift = m_widthOfLift/2

#extrensic parameters
m_heightOfCamera = 5.375 #Need to get actual number from Robot
m_heightOfHighGoalTargetFromCamera = m_heightOfHighGoalTarget - m_heightOfCamera
m_heightOfLiftTargetFromCamera = m_heightOfLiftTarget - m_heightOfCamera
m_degreesAngleOfCamera = 0.0 #Need to get actual number from Robot
m_radiansAngleofCamera = m_degreesAngleOfCamera * (math.pi/180)

#offset parameteres
m_lateralRightOffsetOfLiftCamera = 0.0 #Need to get actual number from Robot
m_forwardOffsetOfLiftCamera = 0.0 #Need to get actual number from Robot
m_lateralRightOffsetOfHighGoalCamera = 0.0 #Need to get actual number from Robot
m_forwardOffsetOfHighGoalCamera = 0.0 #Need to get actual number from Robot
m_lateralRightOffsetOfShooter = 0.0 #Need to get actual number from Robot
m_forwardOffsetOfShooter = 0.0 #Need to get actual number from Robot
m_lateralRightOffsetOfGearPlacer = 0.0 #Need to get actual number from Robot
m_forwardOffsetOfGearPlacer = 0.0 #Need to get actual number from Robot
#m_camera = picamera.PiCamera(resolution = (m_xResolution, m_yResolution))

#print m_centerXOfImage, "and", m_centerYOfImage
#print m_xResolution, "by", m_yResolution

m_camera = picamera.PiCamera(resolution = (m_xResolution, m_yResolution))

def cameraStreamInit():
    #m_camera.resolution = (m_xResolution, m_yResolution)
    m_camera.framerate = 32
    m_camera.shutter_speed = 10000000#800
    m_camera.iso = 100
    m_camera.exposure_mode = 'off'
    m_camera.flash_mode = 'off'
    m_camera.awb_mode = 'off'
    m_camera.drc_strength = 'off'
    m_camera.led = False
    m_camera.awb_gains = 1
    rawCapture = PiRGBArray(m_camera, size=(m_xResolution, m_yResolution))
 
    # allow the camera to warmup
    time.sleep(0.1)
    return rawCapture
    
def getCameraStream(rawCapture):
    for frame in m_camera.capture_continuous(rawCapture, format="bgr",use_video_port=True):
        timestamp = m_camera.timestamp
        image = frame.array
        #aGain = m_camera.analog_gain
        #dGain = m_camera.digital_gain
        #shutterSpeed = m_camera.exposure_speed
        #print
        #print aGain
        #print dGain
        #print shutterSpeed
        #print
        rawCapture.truncate(0)
        h,w = image.shape[:2]
        newCameraMtx, roi = cv2.getOptimalNewCameraMatrix(m_cameraMatrix,m_distCoeffs,(w,h),1,(w,h))
        undistortedImage = cv2.undistort(image, m_cameraMatrix, m_distCoeffs, None, newCameraMtx)
        cv2.imshow('h', undistortedImage)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        return timestamp,undistortedImage
    
def null(x):
    pass

def setupImageWindow():
    #cv2.namedWindow("Original Image")
    cv2.namedWindow("Processed Image")
    cv2.createTrackbar('deltaX', 'Processed Image',0,10,null)
    cv2.createTrackbar('lowDeltaYLimit', 'Processed Image',0,100,null)
    cv2.createTrackbar('highDeltaYLimit', 'Processed Image',0,100,null)
    #cv2.createTrackbar('maxWidth', 'Processed Image',0,500,null)
    #cv2.createTrackbar('maxS', 'Processed Image',0,255,null)
    #cv2.createTrackbar('maxV', 'Processed Image',0,255,null)
  
def findLiftTarget(img):
    #Runs all the filtiration methods to find the Upper High Goal Target
    correctColorImage = filterColors(img,55,210,10,60,255,65)#(img,55,250,10,60,255,65)
    cv2.imshow('Processed Image', correctColorImage)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    preparedImage = prepareImage(correctColorImage)    
    copy = preparedImage.copy() #need to do this because the findContours function alters the source image
    correctNumberOfContoursList = filterContours(copy,4)
    print 'correctNumberOfContoursList: ',len(correctNumberOfContoursList)
    correctSizeList = filterSize(correctNumberOfContoursList,40, 2000,40,2000)
    #drawBoundingBoxes(img, correctSizeList)
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()
    
    print 'correctSizeList: ',len(correctSizeList)
    drawBoundingBoxes(img, correctSizeList)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    correctBlack2WhiteRatioList = filterBlack2WhiteRatio(correctSizeList, preparedImage,0,3)
    print 'correctBlack2WhiteRatioList: ',len(correctBlack2WhiteRatioList)
    drawBoundingBoxes(img, correctBlack2WhiteRatioList)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
    correctLengthToWidthRatioList = filterLength2WidthRatio(correctBlack2WhiteRatioList,0.2,0.8)
    
    print 'correctLengthToWidthRatioList: ',len(correctLengthToWidthRatioList)
    
    
    correctDistanceBetweenTargetsList = filterByOtherTargetLift(correctBlack2WhiteRatioList, 4.4, 25, 30)
    print 'correctDistanceBetweenTargetsList: ',len(correctDistanceBetweenTargetsList)
    
        
    if len(correctLengthToWidthRatioList) == 1:
        conjoinedBloblist = conjoinAnyBlobs(correctSizeList,100,0,100)
        
        for conjoinedBlob in conjoinedBloblist:
            betterFilteredList = correctLengthToWidthRatioList + [conjoinedBlob]
        betterFilteredList = filterByOtherTargetLift(betterFilteredList,4.4,25,30)
        
        return len(betterFilteredList) == 2, betterFilteredList
        
    if len(correctLengthToWidthRatioList) == 2 or len(correctDistanceBetweenTargetsList) == 2:
        firstBoundingBox = correctLengthToWidthRatioList[0]
        secondBoundingBox = correctLengthToWidthRatioList[1]
        #drawBoundingBox(img, firstBoundingBox)
        firstX, firstY, firstWidth, firstHeight = firstBoundingBox
        secondX, secondY, secondWidth, secondHeight = secondBoundingBox
        if firstHeight > secondHeight:
            ret, conjoinedBlob = checkForConjoiningBlobs(secondBoundingBox,correctSizeList, 0.5)
            #print 'conjoinedBlob: ', conjoinedBlob
            if ret:
                filteredList = [conjoinedBlob, firstBoundingBox]
                
            else:
                filteredList = correctLengthToWidthRatioList
                
        else:
            ret, conjoinedBlob = checkForConjoiningBlobs(firstBoundingBox, correctSizeList, 0.5)
            #print 'conjoinedBlob: ', conjoinedBlob
            
            if ret:
                filteredList = [conjoinedBlob, secondBoundingBox]
                
            else:
                filteredList = correctLengthToWidthRatioList
                
        #print
        #print 'filteredList: ', filteredList
        for box in filteredList:
            print box
            
            drawBoundingBoxes(img, filteredList)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
        return True, filteredList
    
    return False, correctBlack2WhiteRatioList
    
    
def findHighGoalTarget(img):
     
    #Runs all the filtiration methods to find the Upper High Goal Target
    correctColorImage = filterColors(img,75,191,48,100,255,255)
    
    preparedImage = prepareImage(correctColorImage)    
    copy = preparedImage.copy() #need to do this because the findContours function alters the source image
    correctNumberOfContoursList = filterContours(copy,4)
    #print len(correctNumberOfContoursList)
    #drawBoundingBoxes(img, correctNumberOfContoursList)
    correctSizeList = filterSize(correctNumberOfContoursList,2,50,30,80)
    #print len(correctSizeList)
    correctWidth = filterWidthHighGoalTarget(correctSizeList)
    #print len(correctWidth)
    #drawBoundingBoxes(img, correctWidth)
    
    correctBlack2WhiteRatioList = filterBlack2WhiteRatio(correctWidth, preparedImage,0,3)
    #print len(correctBlack2WhiteRatioList)
    #drawBoundingBoxes(img, correctBlack2WhiteRatioList)
    
    correctTopHalfBlack2WhiteRatioList = filterTopHalfBlack2WhiteRatio(correctBlack2WhiteRatioList, preparedImage,1,4)
    #print len(correctTopHalfBlack2WhiteRatioList)
  #  drawBoundingBoxes(img, correctTopHalfBlack2WhiteRatioList)
    
    correctLeftHalfBlack2WhiteRatioList = filterLeftHalfBlack2WhiteRatio(correctTopHalfBlack2WhiteRatioList, preparedImage,0,10)
    #print len(correctLeftHalfBlack2WhiteRatioList)
#    drawBoundingBoxes(img, correctLeftHalfBlack2WhiteRatioList)
    #while True:
     #   minRatio = cv2.getTrackbarPos('minRatio','Processed Image')
      #  maxRatio = cv2.getTrackbarPos('maxRatio','Processed Image')
        #minV = cv2.getTrackbarPos('minV','Processed Image')
        #maxH = cv2.getTrackbarPos('maxH','Processed Image')
        #maxS = cv2.getTrackbarPos('maxS','Processed Image')
        #maxV = cv2.getTrackbarPos('maxV','Processed Image')
        
       # correctLeftHalfBlack2WhiteRatioList = filterLeftHalfBlack2WhiteRatio(correctTopHalfBlack2WhiteRatioList, preparedImage,minRatio,maxRatio)
        #drawBoundingBoxes(preparedImage, correctLeftHalfBlack2WhiteRatioList)
        
        
        #key = cv2.waitKey(0)
#        if key == ord('q'): # quit
 #           return None
  #      elif key == ord('g'): # good
   #         break
        # Try again on any other key
    ##print
#    #print minRatio
 #   #print maxRatio
  #  #print 
   # #print 
    ##print 
#    #print 
 #   #print
    #correctDistanceBetweenTargets = filterByDistanceBetweenTargets(correctBlack2WhiteRatioList)
    ##print len(correctDistanceBetweenTargets)
    #drawBoundingBoxes(img, correctDistanceBetweenTargets)
    ##print
    #distanceUShapeIsFromTarget = getDistanceUShapeIsFromTarget(correctTemplateMatchList)
    filteredList = correctSizeList#THIS NEEDS TO BE THE BOUNDING BOX OF THE UPPER PART OF THE HIGH GOAL
    if filteredList == 1:
        return True, filteredList 
    else:
        return False, filteredList
def prepareImage(image):
    #Cancels out very small bits of noice by blurring the image and then eroding it
    #erodedImage = cv2.erode(image,(3,3))
    #erodedImage = cv2.erode(erodedImage,(3,3))
    #erodedImage = cv2.erode(erodedImage,(3,3))
    #erodedImage = cv2.erode(erodedImage,(3,3))
    #erodedImage = cv2.erode(erodedImage,(3,3))
    
    gaussianBlurImage = cv2.GaussianBlur(image,(3,3),1.6)

    return gaussianBlurImage


def filterColors(image,minH,minS,minV,maxH,maxS,maxV):
    #Filters out all colors but green; Returns color filtered image
    HSVImg = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(HSVImg,(minH,minS,minV),(maxH,maxS,maxV))

    return mask

def filterContours(image, numberOfContours):
    #Filters out all "Blobs" with less than "numberOfContours" contours 
    #Returns BOUNDING BOXES of "Blobs" having over 8 contours
    img3,contours,hierarchy = cv2.findContours(image, cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
    goodBoundingBoxes = []
    for box in contours:
        if len(box)>= numberOfContours:
            goodBoundingBoxes = goodBoundingBoxes + [cv2.boundingRect(box)]
    return goodBoundingBoxes
    #Returns BOUNDING BOXES!!!!

def filterSize(goodBoundingBoxes, minHeightSize, maxHeightSize, minWidthSize, maxWidthSize):
    #Filters out "Blobs" that are way too big or way too small
    betterBoundingBoxes = []
    for box in goodBoundingBoxes:
        width =  box[2]
        height =  box[3]
        if minHeightSize < height < maxHeightSize and minWidthSize < width < maxWidthSize:
            betterBoundingBoxes = betterBoundingBoxes + [box]
    return betterBoundingBoxes

def filterWidthHighGoalTarget(goodBoundingBoxes, ratio):
    betterBoundingBoxes = []          
    for box in goodBoundingBoxes:
        width = box[3]
        height = box[2]
        if width < height/ratio:
            betterBoundingBoxes = betterBoundingBoxes +  [box]
    return betterBoundingBoxes

def filterLength2WidthRatio(goodBoundingBoxes, lowLengthToWidthRatio, highLengthToWidthRatio):
    #Filters out all "Blobs" with length to width ratios not between lowLengthToWidthRatio and highLengthToWidthRatio
    betterBoundingBoxes = []          
    for box in goodBoundingBoxes:
        width =  box[2]
        height =  box[3]
        if lowLengthToWidthRatio < (width + 0.0)/ (height+ 0.0) < highLengthToWidthRatio:
            betterBoundingBoxes = betterBoundingBoxes +  [box]
    return betterBoundingBoxes

def filterBlack2WhiteRatio(goodBoundingBoxes, image, blackToWhiteRatioMin, blackToWhiteRatioMax):
    #Filters out all "Blobs" that do not have a ratio of white to black pixels between blackToWhiteRatioMin - blackToWhiteRatioMax 
    betterBoundingBoxes = []
    for box in goodBoundingBoxes:
        x,y,width,height = box
        tempImage = image[y+height/2:y+height, x:x+width]
        numberOfWhitePixels = cv2.countNonZero(tempImage)
        print 'box', box
        if blackToWhiteRatioMin < ((width*(height/2) - numberOfWhitePixels+ 0.0))/(numberOfWhitePixels) < blackToWhiteRatioMax:#number of black pixels for every white pixel
            betterBoundingBoxes = betterBoundingBoxes + [box]
            print "the good one: ", ((width*(height/2) - numberOfWhitePixels+ 0.0))/(numberOfWhitePixels)
        else:
            print "the bad ones: ", ((width*(height/2) - numberOfWhitePixels+ 0.0))/(numberOfWhitePixels)
    
    return betterBoundingBoxes

def filterTopHalfBlack2WhiteRatio(goodBoundingBoxes, image, blackToWhiteRatioMin, blackToWhiteRatioMax):
    #Filters out all "Blobs" that do not have a ratio of white to black pixels between blackToWhiteRatioMin and blackToWhiteRatioMax in the top half of the "Blob" this eliminates upside down and sideways U-shapes
    betterBoundingBoxes = []
    for box in goodBoundingBoxes:
        x,y,width,height = box
        tempImage = image[y:y+height/2, x:x+width]
        numberOfWhitePixels = cv2.countNonZero(tempImage)
        if blackToWhiteRatioMin < ((width*height - numberOfWhitePixels+ 0.0))/(numberOfWhitePixels) < blackToWhiteRatioMax:#number of black pixels for every white pixel
            betterBoundingBoxes = betterBoundingBoxes + [box]
        
    return betterBoundingBoxes

def filterLeftHalfBlack2WhiteRatio(goodBoundingBoxes, image, blackToWhiteRatioMin, blackToWhiteRatioMax):
    #Filters out all "Blobs" that do not have a ratio of white to black pixels between blackToWhiteRatioMin and blackToWhiteRatioMax in the left half of the "Blob" this eliminates upside down and sideways U-shapes
    betterBoundingBoxes = []
    for box in goodBoundingBoxes:
        x,y,width,height = box
        tempImage = image[y:y+height, x:x+width/2]
        numberOfWhitePixels = cv2.countNonZero(tempImage)
        numberOfWhitePixels = cv2.countNonZero(tempImage)
        if blackToWhiteRatioMin < ((width*height - numberOfWhitePixels+ 0.0))/(numberOfWhitePixels) < blackToWhiteRatioMax:#number of black pixels for every white pixel
            betterBoundingBoxes = betterBoundingBoxes + [box]
    return betterBoundingBoxes

#def filterByUShapeTemplateMatch(goodBoundingBoxes, image):
    #Creates and matches a U shape template over "Blobs" that are passed in; Returns blobs that are over 70%(I think %) similar to the template
 #   betterBoundingBoxes = []
  #  for box in goodBoundingBoxes:
   #     x,y,width,height = box
    #    tempImage = image[y:y+height+1, x:x+width+1]
     #   template = np.zeros((width,height,3), np.uint8)
      #  cv2.rectangle(template,(0,0),(height/7,height), (0,255,0),-1)
       # cv2.rectangle(template,(0,height- height/7),(width,height),(0,255,0),-1)
        #cv2.rectangle(template,(width - height/7,0),(width,height),(0,255,0),-1)
#        binaryTemplate = filterColors(template)
 #       results = cv2.matchTemplate(tempImage,binaryTemplate,cv2.TM_CCOEFF_NORMED)
  #      minVal, maxVal, minLoc, maxLoc = cv2.minMaxLoc(results)
   #     if maxVal > .7:
    #        betterBoundingBoxes = betterBoundingBoxes + [box]
    #return betterBoundingBoxes

def filterByDistanceBetweenTargetsHighGoal(goodBoundingBoxes):
    betterBoundingBoxes = []
    for box in goodBoundingBoxes:
        x,y,width,height = box
        for secondBox in goodBoundingBoxes:
            if box == secondBox:
                continue
            secondX,secondY,secondWidth,secondHeight = secondBox
            yDifference = x*y*0.00048
            
            if 0 < secondY - y < yDifference :
                #print "It passes the Y test"
                if secondX - 25 < x <secondX + 25 :
                    #print "It passed the first X test"
                
                    if secondWidth-15 < width < secondWidth + 15 or width-10 < secondWidth < width + 10:
                        #print "It passed the second X test"
                        betterBoundingBoxes = betterBoundingBoxes + [box]
                    else:
                        print "It did not pass the second X test, width = ", width, "and secondWidth = ", secondWidth
                else:
                     print "It did not pass the first X test x was: ", x, "and it had to be between ", secondX - 25, "and", secondX + 25
            else:
                print "It did not pass the first Y test secondY - y was: ", secondY - y, "and the y difference was: ", yDifference
                        
    return betterBoundingBoxes

def filterByOtherTargetLift(goodBoundingBoxes, ratio, yOffset, heightOffset):
    betterBoundingBoxes = []
    if len(goodBoundingBoxes) < 2:
        return goodBoundingBoxes
    for box in goodBoundingBoxes:
        #print 'box: ',box
        x,y,width,height = box
        for secondBox in goodBoundingBoxes:
            
            if box == secondBox:
                continue
            #print 'secondBox: ',secondBox
            secondX,secondY,secondWidth,secondHeight = secondBox
            xDifference = width*ratio #Constant of proportionality of width of the 
            #retro Reflective to the width between the retro targets top left to top left
            #print 'xDifference is:', xDifference  
            if 0 < secondX - x < xDifference:
                #print "passed X test"
                
                if secondY - yOffset < y < secondY + yOffset :
                    #print "passed Y test"
                    if secondHeight-heightOffset < width < secondHeight + heightOffset or height-heightOffset < secondHeight < height + heightOffset:
                        #print "passed Height test"
                        betterBoundingBoxes = betterBoundingBoxes + [box]
                        betterBoundingBoxes = betterBoundingBoxes + [secondBox]
                        
    return betterBoundingBoxes

def conjoinAnyBlobs(otherBoundingBoxesList,ratio):
    betterBoundingBoxes = []
    for box in otherBoundingBoxesList:
        ret, betterBoundingBox = checkForConjoiningBlobs(box,otherBoundingBoxesList,ratio)
        betterBoundingBoxes = betterBoundingBoxes + [betterBoundingBox]
        
    return betterBoundingBoxes

def checkForConjoiningBlobs(goodBoundingBox, otherBoundingBoxesList, ratio):
    betterBoundingBox = []
    x,y,width,height = goodBoundingBox
    #print 'the Length is: ', len(otherBoundingBoxesList)
    ret = False
    for box in otherBoundingBoxesList:
        secondX,secondY,secondWidth,secondHeight = box
        if box == goodBoundingBox:
            continue
        
        if ((x - width*ratio < secondX < x + width*ratio or x + width - width*ratio < secondX + secondWidth < x + width + width*ratio)):
            print "Conjoining blobs: Passed X test"
         
            if y - 1.5*height < secondY < y:
                print 'Conjoining blobs: Passed Y test'
                betterBoundingBox = (x,secondY,width,(y + height) - secondY)
                
                if ret:
                    print "Error: Conjoined more than one blob"
                    return False, betterBoundingBox
                ret = True
        
    return ret, betterBoundingBox


#This is a tuning function

def drawBoundingBoxes (image, goodBoundingBoxes):
    copy = image.copy()
    for box in goodBoundingBoxes:
        x,y,width,height = box
        copy = cv2.rectangle(copy,(x,y),((x + width), (y + height)),(255,0,0), 3)
    small = cv2.resize(copy, (0,0), fx = 0.2, fy = 0.2)
    
    cv2.imshow("Processed Image", small)
    

#These are the Math functions

def getRadiansToTurnFromOpticalAxis(boundingBoxOfTarget):
    x,y,width,height = boundingBoxOfTarget
    distanceFromCenterX = x - m_centerXOfImage
    radiansToTurn = math.atan(distanceFromCenterX/m_focalLengthOfCameraX)
    
    return radiansToTurn

def getRadiansToTurnHighGoalAndDistanceAwayShooter(boundingBoxOfTarget):
    x,y,width,height = boundingBoxOfTarget[0]
    betterBoundingBoxOfTarget = [x + width/2, y, width/2,height]
    radiansToTurnFromCamera = getRadiansToTurnFromOpticalAxis(betterBoundingBoxOfTarget)
    distanceAwayFromHighGoal = getDistanceAwayHighGoal(boundingBoxOfTarget)
    oppositeSide = math.sin(radiansToTurnFromCamera)*distanceAwayFromHighGoal
    adjacentSide = math.cos(radiansToTurnFromCamera)*distanceAwayFromHighGoal
    centerOfRobotAdjacent = adjacentSide + m_forwardOffsetOfHighGoalCamera
    centerOfRobotOppositeSide = oppositeSide + m_lateralRightOffsetOfHighGoalCamera
    centerOfRobotHypotenuse = math.sqrt(centerOfRobotAdjacent*centerOfRobotAdjacent + centerOfRobotOppositeSide*centerOfRobotOppositeSide)
    angleToTurnFromCenterOfRobot = math.atan(centerOfRobotOppositeSide/centerOfRobotAdjacent)
    deltaAngleFromShooter = math.atan(m_lateralRightOffsetOfShooter/centerOfRobotHypotenuse)
    angleToTurnFromShooter = angleToTurnFromCenterOfRobot - deltaAngleFromShooter
    parallelDistanceAway = math.sqrt(centerOfRobotHypotenuse*centerOfRobotHypotenuse - m_lateralRightOffsetOfShooter*m_lateralRightOffsetOfShooter)
    shooterDistanceAway = parallelDistanceAway - m_forwardOffsetOfShooter
    return angleToTurnFromShooter, shooterDistanceAway

def getDistanceAwayHighGoal(boundingBoxOfTarget):
    x,y,width,height = boundingBoxOfTarget[0]
    distanceFromCenterY = m_centerYofImage - y
    elevationAngle = math.atan((distanceFromCenterY)/(m_focalLengthOfCameraY))
    offsetAddedElevationAngle = elevationAngle + m_radiansAngleofCamera
    distanceAwayHighGoalFromCamera = m_heightOfHighGoalTargetFromCamera/math.tan(offsetAddedElevationAngle) #Finding Adjacent; open to change
    return distanceAwayHighGoalFromCamera

def getDistanceAwayLift(boundingBoxOfTarget):
    print "Bounding Box: ", boundingBoxOfTarget
    x,y,width,height = boundingBoxOfTarget
    distanceFromCenterY = m_centerYOfImage - y 
    elevationAngle = math.atan((distanceFromCenterY)/(m_focalLengthOfCameraY))
    offsetAddedElevationAngle = elevationAngle + m_radiansAngleofCamera
    #print offsetAddedElevationAngle*180/math.pi
    #print math.tan(offsetAddedElevationAngle)
    #print
    distanceAwayLift = m_heightOfLiftTargetFromCamera/math.tan(offsetAddedElevationAngle) #Finding Adjacent; open to change
    return distanceAwayLift

def getRadiansToTurnLiftAndDistanceToDriveForwardAndLaterally(boundingBoxesOfTargets):
    firstDistanceAway = getDistanceAwayLift(boundingBoxesOfTargets[0]) #Need this name because boundingBoxesOfTargets does not nessesarily give the targets from left to right
    secondDistanceAway = getDistanceAwayLift(boundingBoxesOfTargets[1])
    if firstDistanceAway > secondDistanceAway:
        furtherTargetHypotenuse = firstDistanceAway
        closerTargetHypotenuse = secondDistanceAway
        furtherBoundingBox = boundingBoxesOfTargets[0]
        closerBoundingBox = boundingBoxesOfTargets[1] 
    else:
        furtherTargetHypotenuse = secondDistanceAway
        closerTargetHypotenuse = firstDistanceAway
        furtherBoundingBox = boundingBoxesOfTargets[1]
        closerBoundingBox = boundingBoxesOfTargets[0]

    if furtherBoundingBox[0] > closerBoundingBox[0]:
        directionToTurn = -1
        directionToSlideLaterally = 1
    else:
        directionToTurn = 1   
        directionToSlideLaterally = -1

    print "furtherTargetHypotenuse: ", furtherTargetHypotenuse
    print "closerTargetHypotenuse: ", closerTargetHypotenuse
    
    angleFromOpticalAxisToFurtherTarget = getRadiansToTurnFromOpticalAxis(furtherBoundingBox)
    angleFromOpticalAxisToCloserTarget = getRadiansToTurnFromOpticalAxis(closerBoundingBox)
    furtherTargetOpposite = math.sin(angleFromOpticalAxisToFurtherTarget)*furtherTargetHypotenuse
    furtherTargetAdjacent = math.cos(angleFromOpticalAxisToFurtherTarget)*furtherTargetHypotenuse
    closerTargetOpposite = math.sin(angleFromOpticalAxisToCloserTarget)*closerTargetHypotenuse
    closerTargetAdjacent = math.cos(angleFromOpticalAxisToCloserTarget)*closerTargetHypotenuse
    centerOfRobotFurtherTargetOpposite = furtherTargetOpposite + m_lateralRightOffsetOfLiftCamera
    centerOfRobotFurtherTargetAdjacent = furtherTargetAdjacent + m_forwardOffsetOfLiftCamera
    centerOfRobotCloserTargetOpposite = closerTargetOpposite + m_lateralRightOffsetOfLiftCamera
    centerOfRobotCloserTargetAdjacent = closerTargetAdjacent + m_forwardOffsetOfLiftCamera
    centerOfRobotFurtherDistanceHypotenuse = math.sqrt(centerOfRobotFurtherTargetOpposite*centerOfRobotFurtherTargetOpposite + centerOfRobotFurtherTargetAdjacent*centerOfRobotFurtherTargetAdjacent)
    centerOfRobotCloserDistanceHypotenuse = math.sqrt(centerOfRobotCloserTargetOpposite*centerOfRobotCloserTargetOpposite + centerOfRobotCloserTargetAdjacent*centerOfRobotCloserTargetAdjacent)
    ratio1 = ((math.pow(centerOfRobotFurtherDistanceHypotenuse, 2) + math.pow(m_widthOfLift, 2) -
              math.pow(centerOfRobotCloserDistanceHypotenuse, 2))/(2*centerOfRobotFurtherDistanceHypotenuse*m_widthOfLift)) #Using law of coesins
    ratio = ((-centerOfRobotCloserDistanceHypotenuse*centerOfRobotCloserDistanceHypotenuse + m_widthOfLift*m_widthOfLift +
        centerOfRobotFurtherDistanceHypotenuse*centerOfRobotFurtherDistanceHypotenuse)/
        (2*m_widthOfLift*centerOfRobotFurtherDistanceHypotenuse))
    #print 'centerOfRobotCloserDistanceHypotenuse ', centerOfRobotCloserDistanceHypotenuse
    #print 'centerOfRobotFurtherDistanceHypotenuse ', centerOfRobotFurtherDistanceHypotenuse
    #print 'ratio1', ratio1
    #print 'ratio', ratio
    oppositeAngle = math.acos(ratio1)

    
    angleOfFurtherTargetToTargetDistance = math.pi - (math.pi/2 + oppositeAngle)
    angleFromRobotToFurtherTarget = math.atan(centerOfRobotFurtherTargetOpposite/centerOfRobotFurtherTargetAdjacent)
    angleFromRobotToCloserTarget = math.atan(centerOfRobotCloserTargetOpposite/centerOfRobotCloserTargetAdjacent)
    angleToTurn = angleOfFurtherTargetToTargetDistance - angleFromRobotToFurtherTarget
    centerOfRobotDistanceToMoveForward = math.cos(angleOfFurtherTargetToTargetDistance)*centerOfRobotFurtherDistanceHypotenuse
    angleToTurn = angleToTurn*directionToTurn
    centerOfRobotDistanceToMoveLaterally = math.sin(angleOfFurtherTargetToTargetDistance)*centerOfRobotFurtherDistanceHypotenuse
    centerOfRobotDistanceToMoveLaterally = centerOfRobotDistanceToMoveLaterally*directionToSlideLaterally
    gearPlacerDistanceToMoveForward = centerOfRobotDistanceToMoveForward - m_forwardOffsetOfGearPlacer
    gearPlacerDistanceToMoveLaterally = centerOfRobotDistanceToMoveLaterally + m_lateralRightOffsetOfGearPlacer
    print 'directionToSlideLaterally', directionToSlideLaterally
    if (centerOfRobotDistanceToMoveLaterally > 0 ):
        gearPlacerDistanceToMoveLaterally = gearPlacerDistanceToMoveLaterally - m_widthOfRetroReflectiveToLift
    else:
        gearPlacerDistanceToMoveLaterally = gearPlacerDistanceToMoveLaterally + m_widthOfRetroReflectiveToLift
        
    return angleToTurn, gearPlacerDistanceToMoveForward, gearPlacerDistanceToMoveLaterally

def initNetworkTables():
    logging.basicConfig(level=logging.DEBUG)
    ip = "10.49.8.77"
    NetworkTables.initialize(server=ip)
    sd = NetworkTables.getTable("VisionProcessing")
    return sd
    
def putDataOnNetworkTablesLift(networkTable, booleanFoundTarget, timestampLift,radiansToTurnLift,distanceToMoveLaterallyLift,distanceToDriveForwardLift):
    networkTable.putBoolean('foundLiftTarget', booleanFoundTarget)
    networkTable.putNumber('radiansToTurnLift', radiansToTurnLift)
    networkTable.putNumber('distanceToMoveLaterallyLift', distanceToMoveLaterallyLift)
    networkTable.putNumber('distanceToDriveForwardLift', distanceToDriveForwardLift)
    networkTable.putNumber('timestampLift', timestampLift)
    
def putDataOnNetworkTablesHighGoal(networkTable, booleanFoundTarget, timestampHighGoal,radiansToTurnHighGoal,distanceAwayHighGoal):
    networkTable.putBoolean('foundHighGoalTarget', booleanFoundTarget)
    networkTable.putNumber('radiansToTurnHighGoal', radiansToTurnHighGoal)
    networkTable.putNumber('distanceAwayHighGoal', distanceAwayHighGoal)
    networkTable.putNumber('timestampHighGoal', timestampHighGoal)
    
def main():
    initializedCameraStream = cameraStreamInit()
    sd = initNetworkTables()
    while True:
        timestamp,cameraStream = getCameraStream(initializedCameraStream)
        #retHighGoal,highGoalTarget = findHighGoalTarget(cameraStream)
        retLift,liftTargets = findLiftTarget(cameraStream)
        if retLift == True:
            radiansToTurnLift, distanceToDriveForwardLift, distanceToMoveLaterallyLift = getRadiansToTurnLiftAndDistanceToDriveForwardAndLaterally(liftTargets)
            putDataOnNetworkTablesLift(sd,True,radiansToTurnLift,timestamp,distanceToMoveLaterallyLift,distanceToDriveForwardLift)
            degreesToTurn = radiansToTurnLift*(180/math.pi)
            print "degreesToTurnLift: ", degreesToTurn
            print 'distanceToMoveLaterallyLift', distanceToMoveLaterallyLift, " Inches"
            print 'distanceToDriveForwardLift', distanceToDriveForwardLift, " Inches"
            
        else:
            putDataOnNetworkTablesLift(sd,False,timestamp,1000,1000,1000)
        print "ready"
        cv2.namedWindow("ready")
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        #if retHighGoal == True:
         #   radiansToTurnHighGoalFromShooter, distanceAwayHighGoalFromShooter = getRadiansToTurnHighGoalAndDistanceAwayShooter(highGoalTarget)
          #  putDataOnNetworkTablesHighGoal(sd,True,timestamp,radiansToTurnHighGoalFromShooter,distanceAwayHighGoalFromShooter)
        #else:
         #   putDataOnNetworkTablesHighGoal(sd,False,timestamp,1000,1000)
        
main()
#rawCapture = cameraStreamInit()
#sd = initNetworkTables()
#timestamp, img = getCameraStream(rawCapture)
#ret, liftTargets = findLiftTarget(img)
##print "liftTargets: ", liftTargets[0][0]

#cv2.destroyAllWindows()
#if ret == True:
 #   radiansToTurnLift, distanceToDriveForwardLift, distanceToMoveLaterallyLift = getRadiansToTurnLiftAndDistanceToDriveForwardAndLaterally(liftTargets)
  #  putDataOnNetworkTablesLift(sd,True,radiansToTurnLift,timestamp, distanceToMoveLaterallyLift,distanceToDriveForwardLift)
   # #print "radiansToTurnLift: ", radiansToTurnLift
    ##print 'distanceToMoveLaterallyLift', distanceToMoveLaterallyLift
    ##print 'distanceToDriveForwardLift', distanceToDriveForwardLift
#else:
 #   putDataOnNetworkTablesLift(sd,False,timestamp,None,None,None)
        

