#! /usr/bin/env python3

import cv2
import sys
import numpy as np
import math

DEBUG = True

def dist(pt1, pt2):
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
            
    #TODO figure out whether to delete or keep
    # longSegments.append(longSegments[0])
    
    # for i in range(1, len(longSegments)):
    #     seg1 = longSegments[i-1]
    #     seg2 = longSegments[i]
        
    #     ang1 = angle(seg1[0], seg1[1])
    #     ang2 = angle(seg2[0], seg2[1])
                
    #     if abs(math.degrees(ang1 - ang2)) > 35:
    #         count += 1
    
    # return count
    return len(longSegments)
    

def isPotentialTarget(contour):
    (x, y), (w, h), a = cv2.minAreaRect(contour)
    
    #first test: is the object big enough?
    if w * h > 5000:
        #second test: is the object smooth enough?
        #a "smooth" object's convex hull length will be roughly equal to the length of the actual contour.
        hull = cv2.convexHull(contour)
        contourLength = cv2.arcLength(contour, True)
        hullLength = cv2.arcLength(hull, True)
        lengthRatio = contourLength / hullLength
        corners = numSegments(contour)
        if abs(1 - lengthRatio) < 0.2:
            #third test: is the aspect ratio good?
            if abs(1 - (w / h)) < 0.3:
                #fourth test: does it have the proper number of sides? TODO: figure out if this test is reliable
                corners = numSegments(contour)
                if corners == 10 or corners == 5 or corners == 8 or corners == 4:
                    return True

    return False


def getTargets(contours, hierarchy, i=0):
    targets = []
    contour = contours[i]
    
    if isPotentialTarget(contour) and hierarchy[i][2] > -1:
        targets.append(contour)
    elif hierarchy[i][2] > -1: #search children if the current contour is not a target
        targets += getTargets(contours, hierarchy, hierarchy[i][2])
    
    if hierarchy[i][0] > -1: #go to next contour
        targets += getTargets(contours, hierarchy, hierarchy[i][0])
        
    return targets


def processImage(img):
    out = img.copy()
    holes = [] #array composed of contours that represent holes in the props
    
    blurred = cv2.blur(img, (5,5))
    edges = cv2.Canny(blurred, 15, 160)
    edges = cv2.dilate(edges, cv2.getStructuringElement(cv2.MORPH_RECT, (3,3)))
    
    contours, _ = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        if w * h > 5000:
            #create blank image to test contour on
            canvas = np.zeros(shape=(img.shape[0], img.shape[1], 1), dtype="uint8")
            canvas = cv2.drawContours(canvas, [contour], 0, (255), 5)
            
            postContours, _ = cv2.findContours(canvas, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            for postContour in postContours:
                if isPotentialTarget(postContour):
                    holes.append(postContour)
            
            #TODO: remove temporary debug code
            # cv2.imshow("canvas", canvas)
            # cv2.waitKey()
    
    #print good contours on output image
    if DEBUG:
        cv2.drawContours(out, holes, -1, (255, 0, 0))
        for contour in holes:
            x, y, w, h = cv2.boundingRect(contour)
            cX = int(x + (w/2))
            cY = int(y + (h/2))
            
            cv2.circle(out, (cX, cY), 2, (0, 255, 0), 2)
            
            #draw all corners that approxpolydp could find?
            arclength = cv2.arcLength(contour, True)
            corners = cv2.approxPolyDP(contour, 0.04 * arclength, True)
            
            for corner in corners:
                cv2.circle(out, (corner[0][0], corner[0][1]), 2, (255, 0, 0), 2)
        
        cv2.imshow("image", out)
        cv2.imshow("edges", edges)
        cv2.waitKey()
    
    return holes


def main():
    for t in ["gman", "bootlegger"]:
        for i in range(1, 6):
            name = "testimages/" + t + "_" + str(i) + ".png"
            img = cv2.imread(name)
            holes = processImage(img)
            
            print(holes)
            
            
if __name__ == '__main__': #start here
    sys.setrecursionlimit(2500) #this is needed because getTargets() uses some monster recursion to go through the contour tree
    main()
    