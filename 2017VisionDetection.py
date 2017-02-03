import cv2
import numpy as np
import os
import math
#from picamera.array import PiRGBArray
#import picamera
import time
import sys
#from networktables import NetworkTables
import logging

#interensic paramaters
m_cameraCalibrationData = np.load('C:\\Users\\admin\\OpenCV Experiments\\CameraCalibrationData.npz')
m_cameraMatrix = m_cameraCalibrationData['mtx'] #Need to load in actual Numbers from Camera Calibration
m_distCoeffs = m_cameraCalibrationData['dist'] #Need to load in actual Numbers from Camera Calibration
m_centerXOfImage = m_cameraMatrix[0,2] #Need to load in actual Numbers from Camera Calibration
m_centerYOfImage = m_cameraMatrix[1,2] #Need to load in actual Numbers from Camera Calibration
m_xResolution = m_centerXOfImage*2 #Need to load in actual Numbers from Camera Calibration
m_yResolution = m_centerYOfImage*2 #Need to load in actual Numbers from Camera Calibration
m_focalLengthOfCameraX = m_cameraMatrix[0,0] #Need to load in actual Numbers from Camera Calibration
m_focalLengthOfCameraY = m_cameraMatrix[1,1] #Need to load in actual Numbers from Camera Calibration
m_horizonLine = 0.9 * m_yResolution # #Need to get actual number from camera

#field parameters
m_heightOfHighGoalTarget = 10.0 #Need to get actual number from manual
m_heightOfLiftTarget = 15.75 #Actual Number From manual
m_widthOfLift = 8.25 #Actual number from manual; Top Left corner of retroReflective to Top right Corner Of RetroReflective
m_widthOfRetroReflectiveToLift = m_widthOfLift/2

#extrensic parameters
m_heightOfCamera = 7.25 #Need to get actual number from Robot
m_heightOfHighGoalTargetFromCamera = m_heightOfHighGoalTarget - m_heightOfCamera
m_heightOfLiftTargetFromCamera = m_heightOfLiftTarget - m_heightOfCamera
m_degreesAngleOfCamera = 0.0 #Need to get actual number from Robot
m_radiansAngleofCamera = m_degreesAngleOfCamera * (math.pi/180)

#offset parameteres
m_lateralRightOffsetOfLiftCamera = 5.0 #Need to get actual number from Robot
m_forwardOffsetOfLiftCamera = 10.0 #Need to get actual number from Robot
m_lateralRightOffsetOfHighGoalCamera = 5.0 #Need to get actual number from Robot
m_forwardOffsetOfHighGoalCamera = 10.0 #Need to get actual number from Robot
m_lateralRightOffsetOfShooter = 5.0 #Need to get actual number from Robot
m_forwardOffsetOfShooter = 10.0 #Need to get actual number from Robot
m_lateralRightOffsetOfGearPlacer = 5.0 #Need to get actual number from Robot
m_forwardOffsetOfGearPlacer = 10.0 #Need to get actual number from Robot

#m_camera = picamera.PiCamera()

def cameraStreamInit():
    m_camera.resolution = (m_xResolution, m_yResolution)
    m_camera.framerate = 32
    m_camera.shutter_speed = 10000
    m_camera.iso = 100
    m_camera.exposure_mode = 'off'
    m_camera.awb_gains = 1
    rawCapture = PiRGBArray(m_camera, size=(m_xResolution, m_yResolution))
 
    # allow the camera to warmup
    time.sleep(0.1)
    return rawCapture
    
def getCameraStream(rawCapture):
    for frame in m_camera.capture_continuous(rawCapture, format="bgr",use_video_port=True):
        timestamp = m_camera.timestamp
        image = frame.array
        cv2.imshow("Image",image)
        aGain = m_camera.analog_gain
        dGain = m_camera.digital_gain
        shutterSpeed = m_camera.exposure_speed
        print
        print aGain
        print dGain
        print shutterSpeed
        print
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        rawCapture.truncate(0)
        undistortedImage = cv2.undistort(image, m_cameraMatrix, m_distCoeffs)
        return timestamp,undistortedImage
    
def null(x):
    pass

def setupImageWindow():
    cv2.namedWindow("Original Image")
    cv2.namedWindow("Processed Image")
    cv2.createTrackbar('minHeight', 'Processed Image',0,512,null)
    cv2.createTrackbar('maxHeight', 'Processed Image',0,512,null)
    cv2.createTrackbar('minWidth', 'Processed Image',0,512,null)
    cv2.createTrackbar('maxWidth', 'Processed Image',0,512,null)
    #cv2.createTrackbar('maxS', 'Processed Image',0,255,null)
    #cv2.createTrackbar('maxV', 'Processed Image',0,255,null)
  
def findLiftTarget(img):
    #Runs all the filtiration methods to find the Upper High Goal Target
    correctColorImage = filterColors(img,92,69,163,112,196,255)
    preparedImage = prepareImage(correctColorImage)    
    copy = preparedImage.copy() #need to do this because the findContours function alters the source image
    correctNumberOfContoursList = filterContours(copy,4)
    print len(correctNumberOfContoursList)
    #drawBoundingBoxes(img, correctNumberOfContoursList)
    correctSizeList = filterSize(correctNumberOfContoursList,37,43,9,24)#,38,43,9,13)
#    while True:
 #       minHeight = cv2.getTrackbarPos('minHeight','Processed Image')
  #      maxHeight = cv2.getTrackbarPos('maxHeight','Processed Image')
   #     minWidth = cv2.getTrackbarPos('minWidth','Processed Image')
    #    maxWidth = cv2.getTrackbarPos('maxWidth','Processed Image')
        #maxS = cv2.getTrackbarPos('maxS','Processed Image')
        #maxV = cv2.getTrackbarPos('maxV','Processed Image')
        
     #   correctLeftHalfBlack2WhiteRatioList = filterSize(correctSizeList, minHeight,maxHeight,minWidth,maxWidth)
      #  drawBoundingBoxes(preparedImage, correctLeftHalfBlack2WhiteRatioList)
        
       # key = cv2.waitKey(0)
        #if key == ord('q'): # quit
         #   return None
#        elif key == ord('g'): # good
 #           break
        # Try again on any other key
  #  print
   # print minHeight
    #print maxHeight
#    print minWidth
 #   print maxWidth
  #  print 
   # print 
    #print
#    print len(correctSizeList)
    #cv2.imshow("Original Image", preparedImage)
    drawBoundingBoxes(img, correctSizeList)
    for box in correctSizeList:
        print "The width is: ", box[2]
    #cv2.waitKey(0)
    correctWidth = filterWidthHighGoalTarget(correctSizeList)
    print len(correctWidth)
    #drawBoundingBoxes(img, correctSizeList)
    
    correctBlack2WhiteRatioList = filterBlack2WhiteRatio(correctWidth, preparedImage,0,3)
    print len(correctBlack2WhiteRatioList)
    #drawBoundingBoxes(img, correctBlack2WhiteRatioList)
    
    correctTopHalfBlack2WhiteRatioList = filterTopHalfBlack2WhiteRatio(correctBlack2WhiteRatioList, preparedImage,1,4)
    print len(correctTopHalfBlack2WhiteRatioList)
  #  drawBoundingBoxes(img, correctTopHalfBlack2WhiteRatioList)
    
    correctLeftHalfBlack2WhiteRatioList = filterLeftHalfBlack2WhiteRatio(correctTopHalfBlack2WhiteRatioList, preparedImage,0,10)
    print len(correctLeftHalfBlack2WhiteRatioList)
#    drawBoundingBoxes(img, correctLeftHalfBlack2WhiteRatioList)
    
    #correctDistanceBetweenTargets = filterByDistanceBetweenTargets(correctBlack2WhiteRatioList)
    #print len(correctDistanceBetweenTargets)
    #drawBoundingBoxes(img, correctDistanceBetweenTargets)
    #print
    #distanceUShapeIsFromTarget = getDistanceUShapeIsFromTarget(correctTemplateMatchList)
    filteredList = correctSizeList
    if filteredList == 2:
        return True, filteredList 
    else:
        return False, filteredList

def findHighGoalTarget(img):
     
    #Runs all the filtiration methods to find the Upper High Goal Target
    correctColorImage = filterColors(img,75,191,48,100,255,255)
    
    preparedImage = prepareImage(correctColorImage)    
    copy = preparedImage.copy() #need to do this because the findContours function alters the source image
    correctNumberOfContoursList = filterContours(copy,4)
    print len(correctNumberOfContoursList)
    #drawBoundingBoxes(img, correctNumberOfContoursList)
    correctSizeList = filterSize(correctNumberOfContoursList,2,50,30,80)
    print len(correctSizeList)
    correctWidth = filterWidthHighGoalTarget(correctSizeList)
    print len(correctWidth)
    #drawBoundingBoxes(img, correctWidth)
    
    correctBlack2WhiteRatioList = filterBlack2WhiteRatio(correctWidth, preparedImage,0,3)
    print len(correctBlack2WhiteRatioList)
    #drawBoundingBoxes(img, correctBlack2WhiteRatioList)
    
    correctTopHalfBlack2WhiteRatioList = filterTopHalfBlack2WhiteRatio(correctBlack2WhiteRatioList, preparedImage,1,4)
    print len(correctTopHalfBlack2WhiteRatioList)
  #  drawBoundingBoxes(img, correctTopHalfBlack2WhiteRatioList)
    
    correctLeftHalfBlack2WhiteRatioList = filterLeftHalfBlack2WhiteRatio(correctTopHalfBlack2WhiteRatioList, preparedImage,0,10)
    print len(correctLeftHalfBlack2WhiteRatioList)
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
    #print
#    print minRatio
 #   print maxRatio
  #  print 
   # print 
    #print 
#    print 
 #   print
    #correctDistanceBetweenTargets = filterByDistanceBetweenTargets(correctBlack2WhiteRatioList)
    #print len(correctDistanceBetweenTargets)
    #drawBoundingBoxes(img, correctDistanceBetweenTargets)
    #print
    #distanceUShapeIsFromTarget = getDistanceUShapeIsFromTarget(correctTemplateMatchList)
    filteredList = correctSizeList#THIS NEEDS TO BE THE BOUNDING BOX OF THE UPPER PART OF THE HIGH GOAL
    if filteredList == 1:
        return True, filteredList 
    else:
        return False, filteredList
def prepareImage(image):
    #Cancels out very small bits of noice by blurring the image and then eroding it
    erodedImage = cv2.erode(image,(3,3))
    erodedImage = cv2.erode(erodedImage,(3,3))
    erodedImage = cv2.erode(erodedImage,(3,3))
    erodedImage = cv2.erode(erodedImage,(3,3))
    #erodedImage = cv2.erode(erodedImage,(3,3))
    #erodedImage = cv2.erode(erodedImage,(3,3))
    
    gaussianBlurImage = cv2.GaussianBlur(erodedImage,(3,3),1.6)

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

def filterWidthHighGoalTarget(goodBoundingBoxes):
    betterBoundingBoxes = []          
    for box in goodBoundingBoxes:
        width = box[3]
        height = box[2]
        if width < height/1.5:
            betterBoundingBoxes = betterBoundingBoxes +  [box]
    return betterBoundingBoxes

def filterLength2WidthRatio(goodBoundingBoxes, lowLengthToWidthRatio, highLengthToWidthRatio):
    #Filters out all "Blobs" with length to width ratios not between lowLengthToWidthRatio and highLengthToWidthRatio
    betterBoundingBoxes = []          
    for box in goodBoundingBoxes:
        width =  box[2]
        height =  box[3]
        if lowLengthToWidthRatio < width/ height < highLengthToWidthRatio:
            betterBoundingBoxes = betterBoundingBoxes +  [box]
    return betterBoundingBoxes

def filterBlack2WhiteRatio(goodBoundingBoxes, image, blackToWhiteRatioMin, blackToWhiteRatioMax):
    #Filters out all "Blobs" that do not have a ratio of white to black pixels between blackToWhiteRatioMin - blackToWhiteRatioMax 
    betterBoundingBoxes = []
    for box in goodBoundingBoxes:
        x,y,width,height = box
        tempImage = image[y:y+height, x:x+width]
        numberOfWhitePixels = cv2.countNonZero(tempImage)
        if blackToWhiteRatioMin < ((width*height - numberOfWhitePixels+ 0.0))/(numberOfWhitePixels) < blackToWhiteRatioMax:#number of black pixels for every white pixel
            betterBoundingBoxes = betterBoundingBoxes + [box]
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
                print "It passes the Y test"
                if secondX - 25 < x <secondX + 25 :
                    print "It passed the first X test"
                
                    if secondWidth-15 < width < secondWidth + 15 or width-10 < secondWidth < width + 10:
                        print "It passed the second X test"
                        betterBoundingBoxes = betterBoundingBoxes + [box]
                    else:
                        print "It did not pass the second X test, width = ", width, "and secondWidth = ", secondWidth
                else:
                     print "It did not pass the first X test x was: ", x, "and it had to be between ", secondX - 25, "and", secondX + 25
            else:
                print "It did not pass the first Y test secondY - y was: ", secondY - y, "and the y difference was: ", yDifference
                        
    return betterBoundingBoxes

def filterByDistanceBetweenTargetsLift(goodBoundingBoxes):
    betterBoundingBoxes = []
    for box in goodBoundingBoxes:
        x,y,width,height = box
        for secondBox in goodBoundingBoxes:
            if box == secondBox:
                continue
            secondX,secondY,secondWidth,secondHeight = secondBox
            xDifference = width*3.125 #Constant of proportionality of the ratio of width of the retro Reflective to the width between the retro targets
            
            if 0 < secondX - x < xDifference:
                print "It passes the X test"
                if secondY - 25 < x <secondY + 25 :
                    print "It passed the first Y test"
                
                    if secondHeight-15 < width < secondHeight + 15 or height-10 < secondHeight < height + 10:
                        print "It passed the second Y test"
                        betterBoundingBoxes = betterBoundingBoxes + [box]
                    else:
                        print "It did not pass the second X test, width = ", width, "and secondWidth = ", secondWidth
                else:
                     print "It did not pass the first X test x was: ", x, "and it had to be between ", secondX - 25, "and", secondX + 25
            else:
                print "It did not pass the first Y test secondY - y was: ", secondY - y, "and the y difference was: ", yDifference
                        
    return betterBoundingBoxes





#This is a tuning function

def drawBoundingBoxes (image, goodBoundingBoxes):
    copy = image.copy()
    for box in goodBoundingBoxes:
        x,y,width,height = box
        cv2.rectangle(copy,(x,y),(x + width, y + height),(0,0,255), 3)
    #cv2.imshow("Processed Image", copy)




#These are the Math functions

def getRadiansToTurnFromOpticalAxis(boundingBoxOfTarget):
    x,y,width,height = boundingBoxOfTarget[0]
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
    x,y,width,height = boundingBoxOfTarget
    distanceFromCenterY = m_centerYOfImage - y 
    elevationAngle = math.atan((distanceFromCenterY)/(m_focalLengthOfCameraY))
    offsetAddedElevationAngle = elevationAngle + m_radiansAngleofCamera
    print offsetAddedElevationAngle*180/math.pi
    print math.tan(offsetAddedElevationAngle)
    print
    distanceAwayLift = m_heightOfLiftTargetFromCamera/math.tan(offsetAddedElevationAngle) #Finding Adjacent; open to change
    print locals()
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
        
    angleFromOpticalAxisToFurtherTarget = getAngleToTurnFromOpticalAxis(furtherBoundingBox)
    angleFromOpticalAxisToCloserTarget = getAngleToTurnFromOpticalAxis(closerBoundingBox)
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
    ratio = ((math.pow(centerOfRobotFurtherDistanceHypotenuse, 2) + math.pow(m_widthOfLift, 2) -
              math.pow(centerOfRobotCloserDistanceHypotenuse, 2))/(2*centerOfRobotFurtherDistanceHypotenuse*m_widthOfLift)) #Using law of coesins
    oppositeAngle = math.acos(ratio)
    angleOfFurtherTargetToTargetDistance = math.pi - (math.pi/2 + oppositeAngle)
    angleFromRobotToFurtherTarget = math.atan(centerOfRobotFurtherTargetOpposite/centerOfRobotFurtherTargetAdjacent)
    angleFromRobotToCloserTarget = math.atan(centerOfRobotCloserTargetOpposite/centerOfRobotCloserTargetAdjacent)
    angleToTurn = angleOfFurtherTargetToTargetDistance - angleFromRobotToFurtherTarget
    centerOfRobotDistanceToMoveForward = math.cos(angleOfFurtherTargetToTargetDistance)*centerOfRobotFurtherDistanceHypotenuse
    angleToTurn = angleToTurn*directionToTurn
    centerOfRobotDistanceToMoveLaterally = math.sin(angleOfFurtherTargetToTargetDistance)*centerOfRobotFurtherDistanceHypotenuse
    centerOfRobotDistanceToMoveLaterally = centerOfRobotDistanceToSlideLaterally*directionToSlideLaterally
    gearPlacerDistanceToMoveForward = centerOfRobotDistanceToMoveForward - m_forwardOffsetOfGearPlacer
    gearPlacerDistanceToMoveLaterally = centerOfRobotDistanceToMoveLaterally + m_lateralRightOffsetOfGearPlacer
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
        retHighGoal,highGoalTarget = findHighGoalTarget(cameraStream)
        retLift,liftTargets = findLiftTarget(cameraStream)
        if retLift == True:
            radiansToTurnLift, distanceToMoveLaterallyLift, distanceToDriveForwardLift = getRadiansToTurnLiftAndDistanceToDriveForwardAndLaterally(liftTargets)
            putDataOnNetworkTablesLift(sd,True,radiansToTurnLift,distanceToMoveLaterallyLift,distanceToDriveForwardLift)
        else:
            putDataOnNetworkTablesLift(sd,False,timestamp,1000,1000,1000)
        if retHighGoal == True:
            radiansToTurnHighGoalFromShooter, distanceAwayHighGoalFromShooter = getRadiansToTurnHighGoalAndDistanceAwayShooter(highGoalTarget)
            putDataOnNetworkTablesHighGoal(sd,True,radiansToTurnHighGoalFromShooter,distanceAwayHighGoalFromShooter)
        else:
            putDataOnNetworkTablesHighGoal(sd,False,timestamp,1000,1000)

print 'X focal length', m_focalLengthOfCameraX
print 'Y focal length', m_focalLengthOfCameraY
print
print 'X resolution is ', m_xResolution, 'Y resolution is ', m_yResolution
print

boundingBox = [m_centerXOfImage, m_centerYOfImage/2 ,10,50]
test = getDistanceAwayLift(boundingBox)

print test

boundingBox = [m_centerXOfImage, m_centerYOfImage ,10,50]
test = getDistanceAwayLift(boundingBox)

print test

boundingBox = [m_centerXOfImage, 0 ,10,50]
test = getDistanceAwayLift(boundingBox)

print test






