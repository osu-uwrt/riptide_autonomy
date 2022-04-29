#! /usr/bin/env python3

import cv2
from matplotlib.pyplot import close
import numpy as np
import math

DEBUG = False

def dist(pt1, pt2): #TODO: this is wrong. get rid of it or fix
    return math.sqrt((pt1[0] * pt1[0]) + (pt2[0] * pt2[0]))


def angle(pt1, pt2):
    y = pt2[1] - pt1[1]
    x = pt2[0] - pt1[0]
    return math.atan(y / x)


#looks for corners in big shapes. this method may not work for corners in small shapes
def numSegments(contour):
    arclength = cv2.arcLength(contour, True)
    corners = cv2.approxPolyDP(contour, 0.02 * arclength, True)
    corners = np.insert(corners, len(corners), corners[0], 0) #add first corner so that it is processed with the last as a segment
        
    longSegments = []
    for i in range(1, len(corners)):
        prev = corners[i-1][0]
        curr = corners[i][0]
        if dist(prev, curr) > 250:
            longSegments.append([prev, curr])
            
    return len(longSegments)
    

def fitsTargetShape(contour):
    (x, y), (w, h), a = cv2.minAreaRect(contour)
    
    #first test: is the object big enough?
    if w * h > 5000:
        #second test: is the object smooth enough?
        #a "smooth" object's convex hull length will be roughly equal to the length of the actual contour.
        hull = cv2.convexHull(contour)
        contourLength = cv2.arcLength(contour, True)
        hullLength = cv2.arcLength(hull, True)
        lengthRatio = contourLength / hullLength
        if abs(1 - lengthRatio) < 0.2:
            #third test: is the aspect ratio good?
            if abs(1 - (w / h)) < 0.3:
                #fourth test: does it have the proper number of sides? TODO: figure out if this test is reliable
                corners = numSegments(contour)
                
                # 10 or 5 corners - star
                # 8 corners - circle
                # 4 corners - trapezoid
                if corners == 10 or corners == 8 or corners == 5 or corners == 4:
                    return True, corners

    return False, -1


def isTarget(contour, imgSize):
    x, y, w, h = cv2.boundingRect(contour)
    if w * h > 5000:
        #create blank image to test contour on
        canvas = np.zeros(shape=(imgSize[0], imgSize[1], 1), dtype="uint8")
        
        #redraw contour thicker to make it more perfect and fill any holes
        canvas = cv2.drawContours(canvas, [contour], 0, (255), 15)
        
        postContours, _ = cv2.findContours(canvas, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for postContour in postContours:
            fitsShape, corners = fitsTargetShape(postContour)
            if fitsShape:
                return True, corners
        
    return False, -1


def getTargets(contours, hierarchy, imgSize, i=0):
    targets = []
    contour = contours[i]
    
    isTarg, corners = isTarget(contour, imgSize)
    if isTarg:
        targets.append((contour, corners))
    elif hierarchy[i][2] > -1: #search children if the current contour is not a target
        targets += getTargets(contours, hierarchy, imgSize, hierarchy[i][2])
    
    if hierarchy[i][0] > -1: #go to next contour
        targets += getTargets(contours, hierarchy, imgSize, hierarchy[i][0])
        
    return targets


def processImage(img: np.ndarray):
    out = img.copy()
 
    blurred = cv2.blur(img, (5,5))
    edges = cv2.Canny(blurred, 15, 160)
    edges = cv2.dilate(edges, cv2.getStructuringElement(cv2.MORPH_RECT, (3,3)))
    
    contours, hierarchy = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    hierarchy = hierarchy[0]
    
    #look for potential targets
    targets = getTargets(contours, hierarchy, (img.shape[0], img.shape[1])) #list of (contour, numCorners)
        
    #draw target contours on output image
    for target, _ in targets:
        cv2.drawContours(out, [target], 0, (0, 0, 255), 2)

        x, y, w, h = cv2.boundingRect(target)
        cv2.circle(out, (int(x + (w/2)), int(y + (h/2))), 2, (0, 255, 0), 2)
    
    #print good contours on output image
    if DEBUG:        
        cv2.imshow("image", out)
        cv2.imshow("edges", edges)
        cv2.waitKey()
    
    return targets


def test():
    for t in ["gman", "bootlegger"]:
        for i in range(1, 6):
            name = "testimages/" + t + "_" + str(i) + ".png"
            img = cv2.imread(name)
            holes = processImage(img)
                        
            
if __name__ == '__main__': #start here
    # sys.setrecursionlimit(2500) 
    test()
    