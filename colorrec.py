import time
import threading
import cv2
import numpy as np
import pytesseract
from PIL import Image
import RPi.GPIO as GPIO
import serial
import numpy as np



# CAMERA 1
#start cameras
cam1 = cv2.VideoCapture(0)
p
#color HSV bounds
cam1_RED_LOWER_0 = np.array([0, 140, 75])
cam1_RED_UPPER_0 = np.array([10, 255, 255])
cam1_RED_LOWER_1 = np.array([165, 140, 75])
cam1_RED_UPPER_1 = np.array([180, 255, 255])

cam1_GREEN_LOWER = np.array([65, 90, 50])
cam1_GREEN_UPPER = np.array([90, 255, 255])

cam1_YELLOW_LOWER = np.array([10, 125, 125])
cam1_YELLOW_UPPER = np.array([35, 255, 255])


#read image from camera
v, img = cam1.read()
img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

#turn colored pixels into their respective colors, everything else white
cam1_red_mask1 = cv2.inRange(img_hsv, cam1_RED_LOWER_0, cam1_RED_UPPER_0)
cam1_red_mask2 = cv2.inRange(img_hsv, cam1_RED_LOWER_1, cam1_RED_UPPER_1)
cam1_red_mask = cv2.bitwise_or(cam1_red_mask1, cam1_red_mask2)
img[cam1_red_mask > 0] = [0, 0, 255]

cam1_yellow_mask = cv2.inRange(img_hsv, cam1_YELLOW_LOWER, cam1_YELLOW_UPPER)
img[cam1_yellow_mask > 0] = [0, 255, 255]

cam1_green_mask = cv2.inRange(img_hsv, cam1_GREEN_LOWER, cam1_GREEN_UPPER)
img[cam1_green_mask > 0] = [0, 255, 0]

img[(cam1_red_mask == 0) & (cam1_yellow_mask == 0) & (cam1_green_mask == 0)] = 255


#blur image and apply contours
blur = cv2.blur(img,(7,7))

gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)

ret, threshold = cv2.threshold(gray,250,255,cv2.THRESH_BINARY_INV)

contours, hierarchy = cv2.findContours(threshold, 
    cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

for c in contours:
    area = cv2.contourArea(c, False)
    print(area)
    if area > 20000:
        print('1')
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.01*peri, True)
        if len(approx) == 4:
            print('2')
            M = cv2.moments(c)
            centerX = int(M['m10']/M['m00'])
            centerY = int(M['m01']/M['m00'])
            
            cam1_green = 0
            cam1_yellow = 0
            cam1_red = 0
            
            for i in range(50):
                for j in range(50):
                    x, y = centerX - 25 + i, centerY - 25 + j
                    b,g,r = img[x, y]
                
                    if b == 0:
                        if g == 255 and r == 0:
                            cam1_green += 1
                        if g == 255 and r == 255:
                            cam1_yellow += 1
                        if g == 0 and r == 255:
                            cam1_red += 1
                            
            if cam1_green == 2500:
                print('cam1 green')
            if cam1_yellow == 2500:
                print('cam1 yellow')
            if cam1_red == 2500:
                print('cam1 red')                
                
            print(cam1_green)
            print(cam1_yellow)
            print(cam1_red)
            
            cam1_green = 0
            cam1_yellow = 0
            cam1_red = 0






# CAMERA 2
#start cameras
cam2 = cv2.VideoCapture(1)

#color HSV bounds
cam2_RED_LOWER_0 = np.array([0, 70, 25])
cam2_RED_UPPER_0 = np.array([10, 255, 255])
cam2_RED_LOWER_1 = np.array([165, 70, 25])
cam2_RED_UPPER_1 = np.array([180, 255, 255])

cam2_GREEN_LOWER = np.array([75, 65, 10])
cam2_GREEN_UPPER = np.array([100, 255, 255])

cam2_YELLOW_LOWER = np.array([10, 70, 25])
cam2_YELLOW_UPPER = np.array([45, 255, 255])


#read image from camera
v, img2 = cam2.read()
img_hsv2 = cv2.cvtColor(img2, cv2.COLOR_BGR2HSV)

#turn colored pixels into their respective colors, everything else white
cam2_red_mask1 = cv2.inRange(img_hsv2, cam2_RED_LOWER_0, cam2_RED_UPPER_0)
cam2_red_mask2 = cv2.inRange(img_hsv2, cam2_RED_LOWER_1, cam2_RED_UPPER_1)
cam2_red_mask = cv2.bitwise_or(cam2_red_mask1, cam2_red_mask2)
img2[cam2_red_mask > 0] = [0, 0, 255]

cam2_yellow_mask = cv2.inRange(img_hsv2, cam2_YELLOW_LOWER, cam2_YELLOW_UPPER)
img2[cam2_yellow_mask > 0] = [0, 255, 255]

cam2_green_mask = cv2.inRange(img_hsv2, cam2_GREEN_LOWER, cam2_GREEN_UPPER)
img2[cam2_green_mask > 0] = [0, 255, 0]

img2[(cam2_red_mask == 0) & (cam2_yellow_mask == 0) & (cam2_green_mask == 0)] = 255


#blur image and apply contours
blur2 = cv2.blur(img2,(3,3))

gray2 = cv2.cvtColor(blur2, cv2.COLOR_BGR2GRAY)

ret, threshold2 = cv2.threshold(gray2,250,255,cv2.THRESH_BINARY_INV)

contours2, hierarchy = cv2.findContours(threshold2, 
    cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

cam2_green = 0
cam2_yellow = 0
cam2_red = 0

for c in contours2:
    peri = cv2.arcLength(c, True)
    approx = cv2.approxPolyDP(c, 0.1*peri, True)
    if len(approx) == 4:
        area = cv2.contourArea(c, False)
        if area > 20000:
            M = cv2.moments(c)
            centerX = int(M['m10']/M['m00'])
            centerY = int(M['m01']/M['m00'])
            for i in range(50):
                for j in range(50):
                    x, y = centerX - 25 + i, centerY - 25 + j
                    b,g,r = img2[x, y]
            
                    if b == 0:
                        if g == 255 and r == 0:
                            cam2_green += 1
                        if g == 255 and r == 255:
                            cam2_yellow += 1
                        if g == 0 and r == 255:
                            cam2_red += 1
                            
            if cam2_green >= 2000:
                print('cam2 green '+str(cam2_green))
            if cam2_yellow >= 2000:
                print('cam2 yellow '+str(cam2_yellow))
            if cam2_red >= 2000:
                print('cam2 red '+str(cam2_red))
                            
            cam2_green = 0
            cam2_yellow = 0
            cam2_red = 0

cv2.imshow('img1', img)
#cv2.imshow('img2', img2)
#cv2.waitKey()




v, img1Let = cam1.read()
v, img2Let = cam2.read()

BLACK_LOWER = 0
BLACK_UPPER = 50

# CAMERA 1
img1_gray = cv2.cvtColor(img1Let, cv2.COLOR_BGR2GRAY)
ret,thresh1 = cv2.threshold(img1_gray,50,255,cv2.THRESH_BINARY_INV)

contours_1, hierarchy = cv2.findContours(thresh1, 
    cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

if len(contours_1) > 0:
    for cnt in contours_1:
        rect = cv2.minAreaRect(cnt)
        x, y = rect[0]
        w, h = rect[1]
        angle = rect[2]
        area = w*h
        if x-(w/2)>0 and x+(w/2)<640 and y-(h/2)>0 and y+(h/2)<480:
            if area > 20000:
                if 1 <= h/w <= 1.6 or 1 <= w/h <= 1.6:
                    rows, cols = thresh1.shape
                    if w<h:
                        rot = cv2.getRotationMatrix2D((x, y), angle, 1)
                        thresh1 = cv2.warpAffine(thresh1, rot, (rows, cols))
                        thresh1 = thresh1[int(y-h/2)-5: int(y+h/2)+5, int(x-w/2)-5: int(x+w/2)+5]
                    else:
                        rot = cv2.getRotationMatrix2D((x, y), angle+90, 1)
                        thresh1 = cv2.warpAffine(thresh1, rot, (rows, cols))
                        thresh1 = thresh1[int(y-w/2)-5: int(y+w/2)+5, int(x-h/2)-5: int(x+h/2)+5]


                    rows, cols = thresh1.shape
                    
                    left_thresh = thresh1[0: rows, 0: int(cols/2)]
                    right_thresh = thresh1[0: rows, int(cols/2): cols]
                    
                    top_thresh = thresh1[0: int(rows/3), 0: cols]
                    mid_thresh = thresh1[int(rows/3): int(2*rows/3), 0: cols]
                    bot_thresh = thresh1[int(2*rows/3): rows, 0: cols]
                    
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
                    
                    
                    if len(L) == 2 and len(R) == 2:
                        print('cam1 S')
                    elif len(T) == 2 and len(M) == 2 and len(B) == 1:
                        print('cam1 U')
                    elif len(T) == 1 and len(M) == 2 and len(B) == 2:
                        print('cam1 U')
                    elif len(T) == 2 and len(M) == 1 and len(B) == 2:
                        print('cam1 H')



# CAMERA 2
img2_gray = cv2.cvtColor(img2Let, cv2.COLOR_BGR2GRAY)
ret,thresh2 = cv2.threshold(img2_gray,50,255,cv2.THRESH_BINARY_INV)

contours_2, hierarchy = cv2.findContours(thresh2, 
    cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

if len(contours_2) > 0:
    for cnt in contours_2:
        rect = cv2.minAreaRect(cnt)
        x, y = rect[0]
        w, h = rect[1]
        angle = rect[2]
        area = w*h
        if x-(w/2)>0 and x+(w/2)<640 and y-(h/2)>0 and y+(h/2)<480:
            if area > 20000:
                if 1 <= h/w <= 1.6 or 1 <= w/h <= 1.6:
                    rows, cols = thresh2.shape
                    if w<h:
                        rot = cv2.getRotationMatrix2D((x, y), angle, 1)
                        thresh2 = cv2.warpAffine(thresh2, rot, (rows, cols))
                        thresh2 = thresh2[int(y-h/2)-5: int(y+h/2)+5, int(x-w/2)-5: int(x+w/2)+5]
                    else:
                        rot = cv2.getRotationMatrix2D((x, y), angle+90, 1)
                        thresh2 = cv2.warpAffine(thresh2, rot, (rows, cols))
                        thresh2 = thresh2[int(y-w/2)-5: int(y+w/2)+5, int(x-h/2)-5: int(x+h/2)+5]


                    rows, cols = thresh2.shape
                    
                    left_thresh = thresh2[0: rows, 0: int(cols/2)]
                    right_thresh = thresh2[0: rows, int(cols/2): cols]
                    
                    top_thresh = thresh2[0: int(rows/3), 0: cols]
                    mid_thresh = thresh2[int(rows/3): int(2*rows/3), 0: cols]
                    bot_thresh = thresh2[int(2*rows/3): rows, 0: cols]
                    
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
                    
                    
                    if len(L) == 2 and len(R) == 2:
                        print('cam2 S')
                    elif len(T) == 2 and len(M) == 2 and len(B) == 1:
                        print('cam2 U')
                    elif len(T) == 1 and len(M) == 2 and len(B) == 2:
                        print('cam2 U')
                    elif len(T) == 2 and len(M) == 1 and len(B) == 2:
                        print('cam2 H')

#cv2.imshow('thresh1', thresh1)
#cv2.imshow('thresh2', thresh2)
cv2.waitKey()

