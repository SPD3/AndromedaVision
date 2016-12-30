import cv2
import numpy as np

def main():
    #Runs all the filtiration methods; It returns the distance the U shape is form the targeted spot on the image
    img = cv2.imread("C:\Users\Public\Pictures\Sample Pictures\UShapeImage6.png")
    preparedImage = prepareImage(img)
    correctColorImage = filterColors(preparedImage)
    copy = correctColorImage.copy() #need to do this because the findContours function alters the source image
    correctNumberOfContoursList = filterContours(copy)
    print len(correctNumberOfContoursList)
    correctSizeList = filterSize(correctNumberOfContoursList)
    print len(correctSizeList)
    correctLength2WidthRatioList = filterLength2WidthRatio(correctSizeList)
    print len(correctLength2WidthRatioList)
    correctBlack2WhiteRatioList = filterBlack2WhiteRatio(correctLength2WidthRatioList, correctColorImage)
    print len(correctBlack2WhiteRatioList)
    correctTopHalfBlack2WhiteRatioList = filterTopHalfBlack2WhiteRatio(correctBlack2WhiteRatioList, correctColorImage)
    print len(correctTopHalfBlack2WhiteRatioList)
    correctLeftHalfBlack2WhiteRatioList = filterLeftHalfBlack2WhiteRatio(correctTopHalfBlack2WhiteRatioList, correctColorImage)
    print len(correctLeftHalfBlack2WhiteRatioList)
    correctTemplateMatchList = filterByTemplateMatch(correctLeftHalfBlack2WhiteRatioList, correctColorImage)
    print len(correctTemplateMatchList)
    print
    correctUShape = getLargestBoundingBox(correctTemplateMatchList)
    distanceUShapeIsFromTarget = getDistanceUShapeIsFromTarget(correctUShape)
    return distanceUShapeIsFromTarget
    
def prepareImage(image):
    #Cancels out very small bits of noice by blurring the image and then eroding it
    GaussianBlurImage = cv2.GaussianBlur(image,(3,3),1.6)
    erodedImage = cv2.erode(GaussianBlurImage,(3,3))
    return erodedImage

def filterColors(image):
    #Filters out all colors but green; Returns color filtered image
    HSVImg = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    img2 = cv2.inRange(HSVImg,(40,0,10),(80,255,255))
    return img2

def filterContours(image):
    #Filters out all "Blobs" with less than 8 contours because a U shape has 8 contours;
    #Returns BOUNDING BOXES of "Blobs" having over 8 contours
    img3,contours,hierarchy = cv2.findContours(image, cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
    goodBoundingBoxes = []
    for box in contours:
        if len(box)>= 8:
            goodBoundingBoxes = goodBoundingBoxes + [cv2.boundingRect(box)]
    return goodBoundingBoxes
    #Returns BOUNDING BOXES!!!!

def filterLength2WidthRatio(goodBoundingBoxes):
    #Filters out all "Blobs" with length to width ratios not between .8 and 1.2
    betterBoundingBoxes = []
    for box in goodBoundingBoxes:
        if 0.8 < box[2]/ box[3] < 1.2:
            betterBoundingBoxes = betterBoundingBoxes +  [box]
    return betterBoundingBoxes

def filterSize(goodBoundingBoxes):
    #Filters out "Blobs" that are way too big or way too small
    betterBoundingBoxes = []
    for box in goodBoundingBoxes:
        width =  box[2]
        height =  box[3]
        if 10 < width < 500 and 10 < height < 500:
            betterBoundingBoxes = betterBoundingBoxes + [box]
    return betterBoundingBoxes
        
def filterBlack2WhiteRatio(goodBoundingBoxes, image):
    #Filters out all "Blobs" that do not have a ratio of white to black pixels between 0.5-0.9:0
    betterBoundingBoxes = []
    for box in goodBoundingBoxes:
        x,y,width,height = box
        tempImage = image[y:y+height, x:x+width]
        numberOfWhitePixels = cv2.countNonZero(tempImage)
        
        if 0.4 < (numberOfWhitePixels + 0.0)/((width*height - numberOfWhitePixels+ 0.0)) < 1.0:#number of white pixels for every black pixel
            betterBoundingBoxes = betterBoundingBoxes + [box]
    return betterBoundingBoxes

def filterTopHalfBlack2WhiteRatio(goodBoundingBoxes, image):
    #Filters out all "Blobs" that do not have a ratio of white to black pixels between .1 and .75 in the top half of the "Blob" this eliminates upside down and sideways U-shapes
    betterBoundingBoxes = []
    for box in goodBoundingBoxes:
        x,y,width,height = box
        tempImage = image[y:y+height/2, x:x+width]
        numberOfWhitePixels = cv2.countNonZero(tempImage)
        whitePixelsPerBlackPixel = (numberOfWhitePixels + 0.0)/(width*(height/2)-numberOfWhitePixels + 0.0)#number of white pixels for every black pixel
        if .3 < whitePixelsPerBlackPixel < .9:
            betterBoundingBoxes = betterBoundingBoxes + [box]
        
    return betterBoundingBoxes

def filterLeftHalfBlack2WhiteRatio(goodBoundingBoxes, image):
    #Filters out all "Blobs" that do not have a ratio of white to black pixels between .6 and .8 in the left half of the "Blob" this eliminates upside down and sideways U-shapes
    betterBoundingBoxes = []
    for box in goodBoundingBoxes:
        x,y,width,height = box
        tempImage = image[y:y+height, x:x+width/2]
        numberOfWhitePixels = cv2.countNonZero(tempImage)
        whitePixelsPerBlackPixel = (numberOfWhitePixels + 0.0)/(width*(height/2)-numberOfWhitePixels + 0.0)#number of white pixels for every black pixel
        if .4 < whitePixelsPerBlackPixel < 1.0:
            betterBoundingBoxes = betterBoundingBoxes + [box]
    return betterBoundingBoxes

def filterByTemplateMatch(goodBoundingBoxes, image):
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

def getLargestBoundingBox(goodBoundingBoxes):
    #gets the largest U shape because the biggest one will be the most direct one to shoot for
    if len(goodBoundingBoxes) == 0:
        return 1000
    elif len(goodBoundingBoxes) == 1:
        return goodBoundingBoxes[0]
    elif len(goodBoundingBoxes) == 2:
        firstArea = goodBoundingBoxes[0][2]*goodBoundingBoxes[0][3]
        secondArea = goodBoundingBoxes[1][2]*goodBoundingBoxes[1][3]
        if firstArea > secondArea:
            return goodBoundingBoxes[0]
        else:
            return goodBoundingBoxes[1]
    elif len(goodBoundingBoxes) == 3:
        firstArea = goodBoundingBoxes[0][2]*goodBoundingBoxes[0][3]
        secondArea = goodBoundingBoxes[1][2]*goodBoundingBoxes[1][3]
        thirdArea = goodBoundingBoxes[2][2]*goodBoundingBoxes[2][3]
        if firstArea > secondArea and firstArea > thirdArea:
            return goodBoundingBoxes[0]
        elif secondArea  > firstArea and secondArea > thirdArea:
            return goodBoundingBoxes[1]
        else:
            return goodBoundingBoxes[2]

def getDistanceUShapeIsFromTarget(UShape):
    #Calculates how far away the Ushape is from the target spot in the image
    if UShape != 1000:
        x = UShape[0]
        target = 512/2
        distance = target - x
        return distance
    else:
        return 1000
    
distance = main()
print distance

