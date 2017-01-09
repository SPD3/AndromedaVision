import cv2
import numpy as np
import os

images = "C:\\Users\\admin\\Pictures\\2017VisionExample\\Vision Images\\LED Boiler"

def findUpperHighGoalTarget(img):
    #Runs all the filtiration methods; It returns the distance the U shape is form the targeted spot on the image
    preparedImage = prepareImage(img)
    correctColorImage = filterColors(preparedImage)
    copy = correctColorImage.copy() #need to do this because the findContours function alters the source image
    correctNumberOfContoursList = filterContours(copy,4)
    print len(correctNumberOfContoursList)
    correctSizeList = filterSize(correctNumberOfContoursList,1,400,3,400)
    print len(correctSizeList)
    correctLength2WidthRatioList = filterLength2WidthRatio(correctSizeList,1.8,2.2)
    print len(correctLength2WidthRatioList)
    correctBlack2WhiteRatioList = filterBlack2WhiteRatio(correctLength2WidthRatioList, correctColorImage,500,100000)
    print len(correctBlack2WhiteRatioList)
    correctTopHalfBlack2WhiteRatioList = filterTopHalfBlack2WhiteRatio(correctBlack2WhiteRatioList, correctColorImage,500,100000)
    print len(correctTopHalfBlack2WhiteRatioList)
    correctLeftHalfBlack2WhiteRatioList = filterLeftHalfBlack2WhiteRatio(correctTopHalfBlack2WhiteRatioList, correctColorImage,500,100000)
    print len(correctLeftHalfBlack2WhiteRatioList)
    print
    #distanceUShapeIsFromTarget = getDistanceUShapeIsFromTarget(correctTemplateMatchList)
    return correctLeftHalfBlack2WhiteRatioList
    
def prepareImage(image):
    #Cancels out very small bits of noice by blurring the image and then eroding it
    gaussianBlurImage = cv2.GaussianBlur(image,(3,3),1.6)
    erodedImage = cv2.erode(gaussianBlurImage,(3,3))
    return gaussianBlurImage

def filterColors(image):
    #Filters out all colors but green; Returns color filtered image
    HSVImg = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    img2 = cv2.inRange(HSVImg,(40,0,10),(80,255,255))
    return img2

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
        
        if blackToWhiteRatioMin < ((width*height - numberOfWhitePixels+ 0.0))/(numberOfWhitePixels + 0.0) < blackToWhiteRatioMax:#number of black pixels for every white pixel
            betterBoundingBoxes = betterBoundingBoxes + [box]
    return betterBoundingBoxes

def filterTopHalfBlack2WhiteRatio(goodBoundingBoxes, image, blackToWhiteRatioMin, blackToWhiteRatioMax):
    #Filters out all "Blobs" that do not have a ratio of white to black pixels between blackToWhiteRatioMin and blackToWhiteRatioMax in the top half of the "Blob" this eliminates upside down and sideways U-shapes
    betterBoundingBoxes = []
    for box in goodBoundingBoxes:
        x,y,width,height = box
        tempImage = image[y:y+height/2, x:x+width]
        numberOfWhitePixels = cv2.countNonZero(tempImage)
        whitePixelsPerBlackPixel = (numberOfWhitePixels + 0.0)/(width*(height/2)-numberOfWhitePixels + 0.0)#number of white pixels for every black pixel
        if blackToWhiteRatioMin < whitePixelsPerBlackPixel < blackToWhiteRatioMax:
            betterBoundingBoxes = betterBoundingBoxes + [box]
        
    return betterBoundingBoxes

def filterLeftHalfBlack2WhiteRatio(goodBoundingBoxes, image, blackToWhiteRatioMin, blackToWhiteRatioMax):
    #Filters out all "Blobs" that do not have a ratio of white to black pixels between blackToWhiteRatioMin and blackToWhiteRatioMax in the left half of the "Blob" this eliminates upside down and sideways U-shapes
    betterBoundingBoxes = []
    for box in goodBoundingBoxes:
        x,y,width,height = box
        tempImage = image[y:y+height, x:x+width/2]
        numberOfWhitePixels = cv2.countNonZero(tempImage)
        whitePixelsPerBlackPixel = (numberOfWhitePixels + 0.0)/(width*(height/2)-numberOfWhitePixels + 0.0)#number of white pixels for every black pixel
        if blackToWhiteRatioMin < whitePixelsPerBlackPixel < blackToWhiteRatioMax:
            betterBoundingBoxes = betterBoundingBoxes + [box]
    return betterBoundingBoxes

def filterByUShapeTemplateMatch(goodBoundingBoxes, image):
    #Creates and matches a U shape template over "Blobs" that are passed in; Returns blobs that are over 70%(I think %) similar to the template
    betterBoundingBoxes = []
    for box in goodBoundingBoxes:
        x,y,width,height = box
        tempImage = image[y:y+height+1, x:x+width+1]
        template = np.zeros((width,height,3), np.uint8)
        cv2.rectangle(template,(0,0),(height/7,height), (0,255,0),-1)
        cv2.rectangle(template,(0,height- height/7),(width,height),(0,255,0),-1)
        cv2.rectangle(template,(width - height/7,0),(width,height),(0,255,0),-1)
        binaryTemplate = filterColors(template)
        results = cv2.matchTemplate(tempImage,binaryTemplate,cv2.TM_CCOEFF_NORMED)
        minVal, maxVal, minLoc, maxLoc = cv2.minMaxLoc(results)
        if maxVal > .7:
            betterBoundingBoxes = betterBoundingBoxes + [box]
    return betterBoundingBoxes

def getDistanceUShapeIsFromTarget(UShape):
    #Calculates how far away the Ushape is from the target spot in the image
    if UShape != 1000:
        x = UShape[0]
        target = 512/2
        distance = target - x
        return distance
    else:
        return 1000

for imgFileName in os.listdir(images):    

    fullFileName = os.path.join(images, imgFileName)
    img = cv2.imread(fullFileName)
    
    cv2.imshow("Image", img)
    if cv2.waitKey(0) == ord('q'):
        cv2.destroyAllWindows()
        break
    
    cv2.destroyAllWindows()
    
    upperHighGoalTarget = findUpperHighGoalTarget(img)
    print len(upperHighGoalTarget)
    print
    print "-----------------------------------"
    
