import cv2
import numpy as np
import os

images = "C:\\Users\\admin\\Pictures\\2017VisionExample\\Vision Images\\LED Boiler"
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
    cv2.imshow("Original Image", preparedImage)
    drawBoundingBoxes(img, correctSizeList)
    for box in correctSizeList:
        print "The width is: ", box[2]
    cv2.waitKey(0)
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
    return correctSizeList
    

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
    while True:
        minRatio = cv2.getTrackbarPos('minRatio','Processed Image')
        maxRatio = cv2.getTrackbarPos('maxRatio','Processed Image')
        #minV = cv2.getTrackbarPos('minV','Processed Image')
        #maxH = cv2.getTrackbarPos('maxH','Processed Image')
        #maxS = cv2.getTrackbarPos('maxS','Processed Image')
        #maxV = cv2.getTrackbarPos('maxV','Processed Image')
        
        correctLeftHalfBlack2WhiteRatioList = filterLeftHalfBlack2WhiteRatio(correctTopHalfBlack2WhiteRatioList, preparedImage,minRatio,maxRatio)
        #drawBoundingBoxes(preparedImage, correctLeftHalfBlack2WhiteRatioList)
        
        
        key = cv2.waitKey(0)
        if key == ord('q'): # quit
            return None
        elif key == ord('g'): # good
            break
        # Try again on any other key
    print
    print minRatio
    print maxRatio
    print 
    print 
    print 
    print 
    print
    #correctDistanceBetweenTargets = filterByDistanceBetweenTargets(correctBlack2WhiteRatioList)
    #print len(correctDistanceBetweenTargets)
    #drawBoundingBoxes(img, correctDistanceBetweenTargets)
    #print
    #distanceUShapeIsFromTarget = getDistanceUShapeIsFromTarget(correctTemplateMatchList)
    return correctSizeList
    
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
        if blackToWhiteRatioMin < ((width*height - numberOfWhitePixels+ 0.0))/(numberOfWhitePixels + 0.0) < blackToWhiteRatioMax:#number of black pixels for every white pixel
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
        if blackToWhiteRatioMin < ((width*height - numberOfWhitePixels+ 0.0))/(numberOfWhitePixels + 0.0) < blackToWhiteRatioMax:#number of black pixels for every white pixel
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

def drawBoundingBoxes (image, goodBoundingBoxes):
    copy = image.copy()
    for box in goodBoundingBoxes:
        x,y,width,height = box
        cv2.rectangle(copy,(x,y),(x + width, y + height),(0,0,255), 3)
    cv2.imshow("Processed Image", copy)

def filterByDistanceBetweenTargets(goodBoundingBoxes):
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
   
#setupImageWindow()

#for imgFileName in os.listdir(images):    

 #   fullFileName = os.path.join(images, imgFileName)
  #  img = cv2.imread(fullFileName)
    
   # cv2.imshow("Original Image", img)
#    if cv2.waitKey(0) == ord('q'):
#        break
    
    
    #HighGoalTarget = findHighGoalTarget(img)
#    if HighGoalTarget is None:
 #       break
  #  print len(HighGoalTarget)
   # print
    #print "-----------------------------------"
#img = cv2.imread("C:\Users\Public\Pictures\StraitOnImage.png")

img1 = cv2.imread("C:\Users\Public\Pictures\StraitOnImage.png")
img2 = cv2.imread("C:\Users\Public\Pictures\Left60DegreesImage.png")
img3 = cv2.imread("C:\Users\Public\Pictures\Right60DegreesImage.png")

correctColorImage1 = filterColors(img1,92,69,163,112,196,255)
correctColorImage2 = filterColors(img2,92,69,163,112,196,255)
correctColorImage3 = filterColors(img3,92,69,163,112,196,255)

cv2.imshow("Original 1",img1)
cv2.imshow("Original 2",img2)
cv2.imshow("Original 3",img3)

cv2.imshow("New 1",correctColorImage1)
cv2.imshow("New 2",correctColorImage2)
cv2.imshow("New 3",correctColorImage3)
cv2.waitKey(0)
cv2.destroyAllWindows()
 
