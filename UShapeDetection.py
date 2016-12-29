import cv2
import numpy as np

def main():
    #Runs all the filtiration methods; In the future it will return the bounding box
    #of the U shape
    img = cv2.imread("C:\Users\Public\Pictures\Sample Pictures\UShapeImage4.png")
    greenImage = filterColors(img)
    correctNumberOfContoursList = filterContours(greenImage)
    correctLength2WidthRatioList = filterLength2WidthRatio(correctNumberOfContoursList)
    print correctLength2WidthRatioList
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
    for x in contours:
        if len(x)>= 8:
            goodBoundingBoxes = goodBoundingBoxes + [cv2.boundingRect(x)]
    return goodBoundingBoxes
    #Returns BOUNDING BOXES!!!!
def filterLength2WidthRatio(goodBoundingBoxes):
    betterBoundingBoxes = []
    for x in goodBoundingBoxes:
        if 0.8 < x[0 ]/ x[1] < 1.2:
            betterBoundingBoxes = betterBoundingBoxes +  [x]
    return betterBoundingBoxes
    
main()

