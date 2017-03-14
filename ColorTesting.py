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
import collections
import ctypes
def filterColors(image,minH,minS,minV,maxH,maxS,maxV):
    #Filters out all colors but green; Returns color filtered image
    HSVImg = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(HSVImg,(minH,minS,minV),(maxH,maxS,maxV))

    return mask

def null(x):
    pass

def setupImageWindow(img):
    cv2.namedWindow("Processed Image")
    cv2.createTrackbar('minH', 'Processed Image',0,255,null)
    cv2.createTrackbar('minS', 'Processed Image',0,255,null)
    cv2.createTrackbar('minV', 'Processed Image',0,255,null)
    cv2.createTrackbar('maxH', 'Processed Image',0,255,null)
    cv2.createTrackbar('maxS', 'Processed Image',0,255,null)
    cv2.createTrackbar('maxV', 'Processed Image',0,255,null)

    while True:
        minH = cv2.getTrackbarPos('minH','Processed Image')
        minS = cv2.getTrackbarPos('minS','Processed Image')
        minV = cv2.getTrackbarPos('minV','Processed Image')
        maxH = cv2.getTrackbarPos('maxH','Processed Image')
        maxS = cv2.getTrackbarPos('maxS','Processed Image')
        maxV = cv2.getTrackbarPos('maxV','Processed Image')
        
        colorImage = filterColors(img, minH,minS,minV, maxH,maxS,maxV)
        small = cv2.resize(colorImage, (0,0), fx = 0.5, fy = 0.5)
        cv2.imshow('Processed Image', small)
        
        key = cv2.waitKey(0)
        if key == ord('q'): # quit
            return None
        elif key == ord('g'): # good
            break
        # Try again on any other key

    print
    print minH
    print minS
    print minV
    print maxH
    print maxS
    print maxV
    print

pics = '/home/pi/test/AndromedaVision/WPI Pics'
for filename in os.listdir(pics):
    fullFileName = os.path.join(pics, filename)
    ##print 'fullFileName', fullFileName
    pic = cv2.imread(fullFileName)
    setupImageWindow(pic)

cv2.destroyAllWindows()
