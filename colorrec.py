import time
import threading
import cv2
import numpy as np
import pytesseract
from PIL import Image
import RPi.GPIO as GPIO
import serial
import numpy as np

#start cameras
cam1 = cv2.VideoCapture(0)
cam2 = cv2.VideoCapture(1)

#color HSV bounds
cam1_RED_LOWER_0 = np.array([0, 140, 75])
cam1_RED_UPPER_0 = np.array([10, 255, 255])
cam1_RED_LOWER_1 = np.array([165, 140, 75])
cam1_RED_UPPER_1 = np.array([180, 255, 255])

cam1_GREEN_LOWER = np.array([65, 90, 50])
cam1_GREEN_UPPER = np.array([90, 255, 255])

cam1_YELLOW_LOWER = np.array([10, 125, 125])
cam1_YELLOW_UPPER = np.array([35, 255, 255])


cam2_RED_LOWER_0 = np.array([0, 70, 25])
cam2_RED_UPPER_0 = np.array([10, 255, 255])
cam2_RED_LOWER_1 = np.array([165, 70, 25])
cam2_RED_UPPER_1 = np.array([180, 255, 255])

cam2_GREEN_LOWER = np.array([75, 65, 10])
cam2_GREEN_UPPER = np.array([100, 255, 255])

cam2_YELLOW_LOWER = np.array([10, 70, 25])
cam2_YELLOW_UPPER = np.array([45, 255, 255])


#colors
def colorRec(index, redLower0, redUpper0, redLower1, redUpper1, greenLower, greenUpper, yellowLower, yellowUpper):
    #read image
    if index == 1:
        v, img = cam1.read()
    if index == 2:
        v, img = cam2.read()
    
    #convert to hsv
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
    #set pixel colors into red, green, yellow, and white
    red_mask1 = cv2.inRange(img_hsv, redLower0, redUpper0)
    red_mask2 = cv2.inRange(img_hsv, redLower1, redUpper1)
    red_mask = cv2.bitwise_or(red_mask1, red_mask2)
    img[red_mask > 0] = [0, 0, 255]

    green_mask = cv2.inRange(img_hsv, greenLower, greenUpper)
    img[green_mask > 0] = [0, 255, 0]
    
    yellow_mask = cv2.inRange(img_hsv, yellowLower, yellowUpper)
    img[yellow_mask > 0] = [0, 255, 255]
    
    img[(red_mask == 0) & (yellow_mask == 0) & (green_mask == 0)] = 255
    
    #apply contours
    blur = cv2.blur(img,(7,7))

    gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)

    ret, thresh = cv2.threshold(gray,250,255,cv2.THRESH_BINARY_INV)

    contours, hierarchy = cv2.findContours(thresh, 
        cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    
    #contours must meet area requirements and have 4 sides
    for c in contours:
        area = cv2.contourArea(c, False)
        if area > 20000:
            peri = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, 0.01*peri, True)
            if len(approx) == 4:
                #crop image around center of contour
                M = cv2.moments(c)
                centerX = int(M['m10']/M['m00'])
                centerY = int(M['m01']/M['m00'])
                
                img = img[centerY-15:centerY+15, centerX-15:centerX+15]
                
                #colors
                green = np.array([0, 255, 0])
                yellow = np.array([0, 255, 255])
                red = np.array([0, 0, 255])
                
                cv2.imshow('img', img)
                
                #count number of colored pixels and return result
                if np.count_nonzero((img == green).all(axis=2)) > 400:
                    print("green")
                
                if np.count_nonzero((img == yellow).all(axis=2)) > 400:
                    print("yellow")
            
                if np.count_nonzero((img == red).all(axis=2)) > 400:
                    print("red")

#letters
def letterRec(index):
    #read image
    if index == 1:
        v, img = cam1.read()
    if index == 2:
        v, img = cam2.read()
    
    #color bounds
    BLACK_LOWER = 0
    BLACK_UPPER = 50
    
    #convert to grayscale and threshold darkest pixels
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(gray,50,255,cv2.THRESH_BINARY_INV)
    
    #find contours
    contours, hierarchy = cv2.findContours(thresh, 
        cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    
    #contours must be fully in the image, meet area requirements, and have a certain height:width ratio
    for c in contours:
        rect = cv2.minAreaRect(c)
        x, y = rect[0]
        w, h = rect[1]
        angle = rect[2]
        area = w*h
        if x-(w/2)>0 and x+(w/2)<640 and y-(h/2)>0 and y+(h/2)<480:
            if area > 20000:
                if 1 <= h/w <= 1.6 or 1 <= w/h <= 1.6:
                    rows, cols = thresh.shape
                    #rotate image so letter is upright
                    if w<h:
                        rot = cv2.getRotationMatrix2D((x, y), angle, 1)
                        thresh = cv2.warpAffine(thresh, rot, (rows, cols))
                        thresh = thresh[int(y-h/2)-5: int(y+h/2)+5, int(x-w/2)-5: int(x+w/2)+5]
                    else:
                        rot = cv2.getRotationMatrix2D((x, y), angle+90, 1)
                        thresh = cv2.warpAffine(thresh, rot, (rows, cols))
                        thresh = thresh[int(y-w/2)-5: int(y+w/2)+5, int(x-h/2)-5: int(x+h/2)+5]
                    
                    #divide image in half horizontally, in thirds vertically
                    rows, cols = thresh.shape

                    left_thresh = thresh[0: rows, 0: int(cols/2)]
                    right_thresh = thresh[0: rows, int(cols/2): cols]

                    top_thresh = thresh[0: int(rows/3), 0: cols]
                    mid_thresh = thresh[int(rows/3): int(2*rows/3), 0: cols]
                    bot_thresh = thresh[int(2*rows/3): rows, 0: cols]
                    
                    #find and count the number of contours in each section
                    contoursL, hierarchy = cv2.findContours(left_thresh,
                        cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
                    contoursR, hierarchy = cv2.findContours(right_thresh,
                        cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

                    contoursT, hierarchy = cv2.findContours(top_thresh,
                        cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
                    contoursM, hierarchy = cv2.findContours(mid_thresh,
                        cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
                    contoursB, hierarchy = cv2.findContours(bot_thresh,
                        cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

                    L = []
                    R = []
                    T = []
                    M = []
                    B = []

                    for c in contoursL:
                        area = cv2.contourArea(c, False)
                        L.append(area)
                    for c in contoursR:
                        area = cv2.contourArea(c, False)
                        R.append(area)
                    for c in contoursT:
                        area = cv2.contourArea(c, False)
                        T.append(area)
                    for c in contoursM:
                        area = cv2.contourArea(c, False)
                        M.append(area)
                    for c in contoursB:
                        area = cv2.contourArea(c, False)
                        B.append(area)
                    
                    #ignore contours that are too small
                    for area in L:
                        if area < max(L)/4:
                            L.remove(area)
                    for area in R:
                        if area < max(R)/4:
                            R.remove(area)
                    for area in T:
                        if area < max(T)/4:
                            T.remove(area)
                    for area in M:
                        if area < max(M)/4:
                            M.remove(area)
                    for area in B:
                        if area < max(B)/4:
                            B.remove(area)
                    
                    #return result
                    if len(L) == 2 and len(R) == 2:
                        print('cam1 S')
                    elif len(T) == 2 and len(M) == 2 and len(B) == 1:
                        print('cam1 U')
                    elif len(T) == 1 and len(M) == 2 and len(B) == 2:
                        print('cam1 U')
                    elif len(T) == 2 and len(M) == 1 and len(B) == 2:
                        print('cam1 H')

                    cv2.imshow('thresh', thresh)


#run color recognition
colorRec(1, cam1_RED_LOWER_0, cam1_RED_UPPER_0, cam1_RED_LOWER_1, cam1_RED_UPPER_1, cam1_GREEN_LOWER, cam1_GREEN_UPPER, cam1_YELLOW_LOWER, cam1_YELLOW_UPPER)
colorRec(2, cam2_RED_LOWER_0, cam2_RED_UPPER_0, cam2_RED_LOWER_1, cam2_RED_UPPER_1, cam2_GREEN_LOWER, cam2_GREEN_UPPER, cam2_YELLOW_LOWER, cam2_YELLOW_UPPER)

#run letter recognition
letterRec(1)
letterRec(2)

cv2.waitKey()

