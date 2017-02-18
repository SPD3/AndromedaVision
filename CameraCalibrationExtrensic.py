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

m_xResolution = 2656 
m_yResolution = 1328
m_cameraCalibrationData = np.load('/home/pi/test/AndromedaVision/CameraCalibrationData.npz')
m_cameraMatrix = np.matrix([[  2.04031106e+03,   0.00000000e+00,   1.36688532e+03],
 [  0.00000000e+00,   2.04279929e+03,   6.65064554e+02],
 [  0.00000000e+00,   0.00000000e+00,   1.00000000e+00]])
m_distCoeffs = np.matrix([[ 0.18141488, -0.47026778, -0.00274879, -0.00065564,  0.33265707]])
print m_cameraMatrix
print np.load('/home/pi/Desktop/mtx.npy')
print m_distCoeffs
print np.load('/home/pi/Desktop/dist.npy')
#field parameters
m_heightOfHighGoalTarget = 10.0 #Need to get actual number from manual
m_heightOfLiftTarget = 15.75 #Actual Number From manual
m_widthOfLift = 8.25 #Actual number from manual; Top Left corner of retroReflective to Top right Corner Of RetroReflective
m_widthOfRetroReflectiveToLift = m_widthOfLift/2
m_camera = picamera.PiCamera(resolution = (m_xResolution, m_yResolution))

def cameraStreamInit():
    #m_camera.resolution = (m_xResolution, m_yResolution)
    m_camera.framerate = 10
    m_camera.shutter_speed = 900
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
        rawCapture.truncate(0)
        #h,w = image.shape[:2]
        #newCameraMtx, roi = cv2.getOptimalNewCameraMatrix(m_cameraMatrix,m_distCoeffs,(w,h),1,(w,h))
        print 'undistorting'
        #undistortedImage = cv2.undistort(image, m_cameraMatrix, m_distCoeffs, None, newCameraMtx)
        print 'undistorted'    
        #cv2.imshow('h', undistortedImage)
        #cv2.waitKey(0)
        #cv2.destroyAllWindows()
        return timestamp,image

def findLiftTarget(img):
    #Runs all the filtiration methods to find the Upper High Goal Target
    correctColorImage = filterColors(img,55,250,10,60,255,65)
    #cv2.imshow('Processed Image', correctColorImage)
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()
    preparedImage = prepareImage(correctColorImage)    
    copy = preparedImage.copy() #need to do this because the findContours function alters the source image
    correctNumberOfContoursList = filterContours(copy,4)
    print 'correctNumberOfContoursList: ',len(correctNumberOfContoursList)
    #drawBoundingBoxes(img, correctNumberOfContoursList)
    #cv2.waitKey()
    #cv2.destroyAllWindows()
    correctSizeList = filterSize(correctNumberOfContoursList,10, 2000,10,2000)
    #drawBoundingBoxes(img, correctSizeList)
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()
    
    print 'correctSizeList: ',len(correctSizeList)
    drawBoundingBoxes(img, correctSizeList)
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()
    correctBlack2WhiteRatioList = filterBlack2WhiteRatio(correctSizeList, preparedImage,0,3)
    print 'correctBlack2WhiteRatioList: ',len(correctBlack2WhiteRatioList)
    drawBoundingBoxes(img, correctBlack2WhiteRatioList)
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()
    
    correctLengthToWidthRatioList = filterLength2WidthRatio(correctBlack2WhiteRatioList,0.2,0.6)
    
    print 'correctLengthToWidthRatioList: ',len(correctLengthToWidthRatioList)
    
    
    #correctDistanceBetweenTargetsList = filterByOtherTargetLift(correctBlack2WhiteRatioList, 4.4, 25, 30)
    #print 'correctDistanceBetweenTargetsList: ',len(correctDistanceBetweenTargetsList)
    
        
    if len(correctLengthToWidthRatioList) != 2 and len(correctLengthToWidthRatioList) != 0:
        conjoinedBloblist = conjoinAnyBlobs(correctSizeList,0.5)
        betterConjoinedBloblist = []
        print 'conjoinedBloblist', conjoinedBloblist
        for conjoinedBlob in conjoinedBloblist:
            print 'len(conjoinedBlob): ',len(conjoinedBlob)
            if len(conjoinedBlob) == 4:
                betterConjoinedBloblist = betterConjoinedBloblist + [conjoinedBlob]
        if len(betterConjoinedBloblist) != 0:
            if betterConjoinedBloblist == 0:
                betterFilteredList = correctLengthToWidthRatioList
                print 'betterConjoinedBloblist == 0'
            for conjoinedBlob in betterConjoinedBloblist:
                betterFilteredList = correctLengthToWidthRatioList + [conjoinedBlob]
                print "adding: ", conjoinedBlob
        else:
            betterFilteredList = correctLengthToWidthRatioList
            print "here"
        print 'len(betterFilteredList): ', len(betterFilteredList)
        print '[betterFilteredList]: ', [betterFilteredList]
        betterFilteredList = filterByOtherTargetLift(betterFilteredList,5,100,65)
        print '1'
        print 'final result: ', len(betterFilteredList)
        drawBoundingBoxes(img, betterFilteredList)
        #cv2.waitKey(0)
        #cv2.destroyAllWindows()
        return len(betterFilteredList) == 2, betterFilteredList
        
    if len(correctLengthToWidthRatioList) == 2 :
        firstBoundingBox = correctLengthToWidthRatioList[0]
        secondBoundingBox = correctLengthToWidthRatioList[1]
        #drawBoundingBox(img, firstBoundingBox)
        firstX, firstY, firstWidth, firstHeight = firstBoundingBox
        secondX, secondY, secondWidth, secondHeight = secondBoundingBox
        if firstHeight > secondHeight:
            ret, conjoinedBlob = checkForConjoiningBlobs(secondBoundingBox,correctNumberOfContoursList, 0.5)
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
        print 'filteredList 1: ', filteredList
        drawBoundingBoxes(img, filteredList)
        #cv2.waitKey(0)
        #cv2.destroyAllWindows()
        filteredList = filterByOtherTargetLift(filteredList, 5, 100, 65)
        print 'filteredList 2: ', filteredList
        #print
        #print 'filteredList: ', filteredList
        #for box in filteredList:
         #   print box
            
          #  drawBoundingBoxes(img, filteredList)
           # cv2.waitKey(0)
            #cv2.destroyAllWindows()
        
        if len(filteredList) == 2:
            print 'YES final result: ', len(filteredList)
            return True, filteredList
        
    print 'final result: 0'
    return False, correctBlack2WhiteRatioList
def prepareImage(image):
    #Cancels out very small bits of noice by blurring the image and then eroding it
    #erodedImage = cv2.erode(image,(3,3))
    #erodedImage = cv2.erode(erodedImage,(3,3))
    #erodedImage = cv2.erode(erodedImage,(3,3))
    #erodedImage = cv2.erode(erodedImage,(3,3))
    #erodedImage = cv2.erode(erodedImage,(3,3))
    
    #gaussianBlurImage = cv2.GaussianBlur(image,(3,3),1.6)

    return image


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
        if len(box) == 0:
            print 'uh oh 1'
            continue
        x,y,width,height = box
        
        for secondBox in goodBoundingBoxes:
            
            if box == secondBox:
                
                continue
            if len(secondBox) == 0:
                print 'uh oh 3'
                continue
            
            print 'secondBox: ',secondBox
            print 'len(secondBox): ', len(secondBox)
            print 'len(goodBoundingBoxes): ',len(goodBoundingBoxes)
            secondX,secondY,secondWidth,secondHeight = secondBox
            xDifference = width*ratio #Constant of proportionality of width of the 
            #retro Reflective to the width between the retro targets top left to top left
            print 'xDifference is:', xDifference
            print 'comparing: ', box, 'and', secondBox
            if 0 < secondX - x < xDifference:
                print "passed X test"
                
                if secondY - yOffset < y < secondY + yOffset :
                    print "passed Y test"
                    if secondHeight-heightOffset < height < secondHeight + heightOffset or height-heightOffset < secondHeight < height + heightOffset:
                        print "passed Height test"
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
        copy = cv2.rectangle(copy,(x,y),((x + width), (y + height)),(255,0,0), 100)
    small = cv2.resize(copy, (0,0), fx = 0.2, fy = 0.2)
    
    #cv2.imshow("Processed Image", small)

pictures = "/home/pi/Pictures/ExtrensicCameraCalibrationPictures"

objPoints = np.matrix([[-5.125,59.75,15.75],[-3.125,59.75,10.75],[-5.125,59.75,10.75],[-3.125,59.75,15.75],[3.125,59.75,15.75],[5.125,59.75,10.75],
                       [3.125,59.75,10.75],[5.125,59.75,15.75]]) #HARD CODE IN THESE VALUES
#objPoints = np.matrix([[0,20.0,15.75],[2,20.0,15.75],[5.25,20.0,15.75],[10.25,20.0,15.75],[0,20.0,10.75],
                      #[2,20.0,10.75],[8.25,20.0,10.75],[10.25,20.0,10.75]])
def calibrateCameraExtrensic():
    imgpoints = []#np.empty((2,8))
    for filename in os.listdir(pictures):
        fullFileName = os.path.join(pictures, filename)
        picture = cv2.imread(fullFileName)
        ret, targets = findLiftTarget(picture)
        print 'targets', targets
        for target in targets:
            offset = 10
            x,y,width,height = target
            tempImageCorner1 = picture[y - offset:y+offset, x-offset:x+offset]
            tempImageCorner2 = picture[y + height - offset:y+height+ offset, x+ width-offset:x+width+offset]
            tempImageCorner3 = picture[y + height - offset:y+height+ offset, x-10:x+offset]
            tempImageCorner4 = picture[y - offset:y+ offset, x + width - offset:x+width+ offset]
            #tempCornerImage1
            #correctColorImage = filterColors(tempImage,50,60,100,100,190,255)#(img,55,250,10,60,255,65)
    
            #print tempImage
            grayTempCorner1 = cv2.cvtColor(tempImageCorner1, cv2.COLOR_BGR2GRAY)
            grayTempCorner2 = cv2.cvtColor(tempImageCorner2, cv2.COLOR_BGR2GRAY)
            grayTempCorner3 = cv2.cvtColor(tempImageCorner3, cv2.COLOR_BGR2GRAY)
            grayTempCorner4 = cv2.cvtColor(tempImageCorner4, cv2.COLOR_BGR2GRAY)
            
            #cv2.imshow('binary',correctColorImage)
            #cv2.waitKey()
            grayTempCorner1 = np.float32(grayTempCorner1)
            grayTempCorner2 = np.float32(grayTempCorner2)
            grayTempCorner3 = np.float32(grayTempCorner3)
            grayTempCorner4 = np.float32(grayTempCorner4)
            
            dst1 = cv2.cornerHarris(grayTempCorner1, 1, 7, 2)
            #print 'dst1', dst1
            dst2 = cv2.cornerHarris(grayTempCorner2, 1, 7, 2)
            dst3 = cv2.cornerHarris(grayTempCorner3, 1, 7, 2)
            dst4 = cv2.cornerHarris(grayTempCorner4, 1, 7, 2)
            
            ret, dst1 = cv2.threshold(dst1, 0.01*dst1.max(),255,0)
            ret, dst2 = cv2.threshold(dst2, 0.01*dst2.max(),255,0)
            ret, dst3 = cv2.threshold(dst3, 0.01*dst3.max(),255,0)
            ret, dst4 = cv2.threshold(dst4, 0.01*dst4.max(),255,0)
            
            dst1 = np.uint8(dst1)
            dst2 = np.uint8(dst2)
            dst3 = np.uint8(dst3)
            dst4 = np.uint8(dst4)
            
            ret1, labels1, stats1, centroids1 = cv2.connectedComponentsWithStats(dst1)
            
            ret2, labels2, stats2, centroids2 = cv2.connectedComponentsWithStats(dst2)
            ret3, labels3, stats3, centroids3 = cv2.connectedComponentsWithStats(dst3)
            ret4, labels4, stats4, centroids4 = cv2.connectedComponentsWithStats(dst4)
            
            #print 'centroids', centroids
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.001)
            corners1 = cv2.cornerSubPix(grayTempCorner1,np.float32(centroids1) , (5,5), (-1,-1), criteria)
            corners2 = cv2.cornerSubPix(grayTempCorner2,np.float32(centroids2) , (5,5), (-1,-1), criteria)
            corners3 = cv2.cornerSubPix(grayTempCorner3,np.float32(centroids3) , (5,5), (-1,-1), criteria)
            corners4 = cv2.cornerSubPix(grayTempCorner4,np.float32(centroids4) , (5,5), (-1,-1), criteria)

            print 'corner1[0][0]',corners1[0][0]
            corners1 = [[corners1[0][0] + x - offset, corners1[0][1] + y -offset]]
            corners2 = [[corners2[0][0] + x - offset + width,corners2[0][1]+ y - offset + height]]
            corners3 = [[corners3[0][0] + x - offset ,corners3[0][1]+ height + y - offset]]
            corners4 = [[corners4[0][0] + x - offset + width ,corners4[0][1] + y - offset]]
            print 'len corners', len(corners1) + len(corners2) + len(corners3) + len(corners4)
            #np.append(imgpoints,corners)
            #imgpoints.append(corners)
            if imgpoints == []:
                imgpoints = corners1
                imgpoints = np.append(imgpoints,corners2,0)
                imgpoints = np.append(imgpoints,corners3,0)
                imgpoints = np.append(imgpoints,corners4,0)
                print 'imgpoints', imgpoints
            else:
                imgpoints = np.append(imgpoints, corners1,0)
                imgpoints = np.append(imgpoints, corners2,0)
                imgpoints = np.append(imgpoints, corners3,0)
                imgpoints = np.append(imgpoints, corners4,0)
                print 'imgpoints', imgpoints
                
        
            print 'width', width
            print 'height', height
            
            
            #res = np.hstack((centroids1, corners1))
            #res = np.int0(res)
            #tempImage[res[:,1],res[:,0]] = [0,0,255]
            #tempImageCorner1[res[:,3],res[:,2]] = [0,255,0]
            #small = cv2.resize(picture, (0,0), fx = 1, fy = 1)
            #small = cv2.dilate(tempImage,(3,3))
            #small = cv2.dilate(small,(3,3))
            
            #cv2.imshow('Corners', tempImageCorner1)
            #cv2.waitKey()
    cv2.destroyAllWindows()
    print 'objPoints', objPoints
    print 'imgpoints', imgpoints
    ret, rvec, tvec = cv2.solvePnP(objPoints, imgpoints, m_cameraMatrix, m_distCoeffs)
    #newCameraMatrix,newRvecs,newTvecs, rotMatX, rotMatY, RotMatZ, eulerAngles = cv2.decomposeProjectionMatrix(imgpoints, m_cameraMatrix, rvec, tvec)
    
    #print 'newCameraMatrix ', newCameraMatrix
    #print 'm_cameraMatrix: ', m_cameraMatrix
    R, jacobian = cv2.Rodrigues(rvec)
    return R, tvec, rvec

def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6

def rotationMatrixToEulerAngles(R):
    assert(isRotationMatrix(R))

    sy = math.sqrt(R[0,0]*R[0,0] + R[1,0]*R[1,0])
    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2,1], R[2,2])
        y = math.atan2(-R[2,0],sy)
        z = math.atan2(R[1,0], R[0,0])
    else:
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0],sy)
        z = 0
    return np.array([x,y,z])

R,tvec, rvec = calibrateCameraExtrensic()
if isRotationMatrix(R):
    print 'rvec', rvec
    print 'tvec', tvec
    eulerAngles = rotationMatrixToEulerAngles(R)
    print 'eulerAngles', eulerAngles #,np.linalg.norm(rvec),math.pi/2
    

np.save(('/home/pi/Desktop/R.npy'), R)
np.save(('/home/pi/Desktop/tvec.npy'), tvec)

