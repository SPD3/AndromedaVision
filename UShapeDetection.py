import cv2
import numpy as np

def main():
    #Runs all the filtiration methods; In the future it will return the bounding box
    #of the U shape
    img = cv2.imread("C:\Users\Public\Pictures\Sample Pictures\UShapeImage5.png")
    preparedImage = prepareImage(img)
    correctColorImage = filterColors(preparedImage)
    copy=correctColorImage.copy() #need to do this because the findContours function alters the source image
    correctNumberOfContoursList = filterContours(copy)
    correctLength2WidthRatioList = filterLength2WidthRatio(correctNumberOfContoursList)
    correctBlack2WhiteRatioList = filterBlack2WhiteRatio(correctLength2WidthRatioList, correctColorImage)
    correctTopHalfBlack2WhiteRatioList = filterTopHalfBlack2WhiteRatio(correctBlack2WhiteRatioList, correctColorImage)
    correctLeftHalfBlack2WhiteRatioList = filterLeftHalfBlack2WhiteRatio(correctTopHalfBlack2WhiteRatioList, correctColorImage)
    correctTemplateMatchList = filterByTemplateMatch(correctLeftHalfBlack2WhiteRatioList, correctColorImage)
    
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
    #Filter out all "Blobs" with length to width ratios not between .8 and 1.2
    betterBoundingBoxes = []
    for box in goodBoundingBoxes:
        if 0.8 < box[2]/ box[3] < 1.2:
            betterBoundingBoxes = betterBoundingBoxes +  [box]
    return betterBoundingBoxes

def filterBlack2WhiteRatio(goodBoundingBoxes, image):
    #Filters out all "Blobs" that do not have a ratio of white to black pixels between 0.6-0.8:0
    betterBoundingBoxes = []
    for box in goodBoundingBoxes:
        x,y,width,height = box
        tempImage = image[y:y+height, x:x+width]
        numberOfWhitePixels = cv2.countNonZero(tempImage)
        
        if 0.5 < (numberOfWhitePixels + 0.0)/((width*height - numberOfWhitePixels+ 0.0)) < 0.9:#number of white pixels for every black pixel
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
        if .1 < whitePixelsPerBlackPixel < .75:
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
        if .5 < whitePixelsPerBlackPixel < .9:
            betterBoundingBoxes = betterBoundingBoxes + [box]
    return betterBoundingBoxes

def filterByTemplateMatch(goodBoundingBoxes, image):
    betterBoundingBoxes = []
    for box in goodBoundingBoxes:
        x,y,width,height = box
        tempImage = image[y:y+height, x:x+width]
        template = np.zeros((512,512,3), np.uint8)
        cv2.rectangle(template,(x,y),(int(height/3.5),height + y), (0,255,0),-1)
        cv2.rectangle(template,(x,y+height- int(height/7)),(x+width,y+height),(0,255,0),-1)
        cv2.rectangle(template,(x + width - int(height/7),y),(x+width,y+height),(0,255,0),-1)
        binaryTemplate = filterColors(template)
        
        
main()

