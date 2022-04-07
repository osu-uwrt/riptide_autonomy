#! /usr/bin/env python3

from http.client import HTTP_VERSION_NOT_SUPPORTED
import cv2
import sys
from math import sqrt

from numpy import isposinf

DEBUG = False

def dist(pt1, pt2):
    return sqrt((pt1[0] * pt1[0]) + (pt2[0] * pt2[0]))


def isPotentialTarget(contour):
    x, y, w, h = cv2.boundingRect(contour)
                
    #first test: is the object big enough?
    if w * h > 500:
        #second test: is the object smooth enough?
        #a "smooth" object's convex hull length will be roughly equal to the length of the actual contour.
        hull = cv2.convexHull(contour)
        contourLength = cv2.arcLength(contour, True)
        hullLength = cv2.arcLength(hull, True)
        lengthRatio = contourLength / hullLength
        if abs(1 - lengthRatio) < 0.2:
            #third test: is the aspect ratio good?
            if abs(1 - (w / h)) < 0.2:
                return True

    return False


def getTargets(contours, hierarchy, i=0):
    targets = []
    contour = contours[i]
    
    if isPotentialTarget(contour):
        #TODO: do this better. should check all contours on that child level for target-ness and there should be exactly one that is a potential target. dont just test the first child
        if hierarchy[i][2] > -1 and isPotentialTarget(contours[ hierarchy[i][2] ]): #check that first (should be the only) child is also a target
            targets.append(contour)
    elif hierarchy[i][2] > -1: #search children if the current contour is not a target
        targets += getTargets(contours, hierarchy, hierarchy[i][2])
    
    if hierarchy[i][0] > -1:
        targets += getTargets(contours, hierarchy, hierarchy[i][0])
        
    return targets


def main():
    types = ["gman", "bootlegger"]
    for t in types:
        for i in range(1, 6):
            name = "testimages/" + t + "_" + str(i) + ".png"
            img = cv2.imread(name)
            out = img.copy()
            
            _, img = cv2.threshold(img, 50, 255, cv2.THRESH_BINARY)
            img = cv2.blur(img, (5, 5))
            bin = cv2.inRange(img, (0, 0, 100), (100, 100, 255))
            # edges = cv2.Canny(bin, 100, 200)
            edges = bin.copy()
            contours, hierarchy = cv2.findContours(edges, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
            hierarchy = hierarchy[0]
            
            edgesColor = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
            
            n = 0
            approvedContours = getTargets(contours, hierarchy)
                            
            for contour in approvedContours:
                #image markup stuff
                x, y, w, h = cv2.boundingRect(contour)
                n = n + 1
                                
                cv2.drawContours(edgesColor, contour, -1, (0, 255, 0), 2)
                cenX = x + (w/2)
                cenY = y + (h/2)
                cv2.circle(out, (int(cenX), int(cenY)), 1, (0, 0, 255), 2)
                cv2.putText(out, str(n), (int(cenX), int(cenY)), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0))  
            
            #for debug stuff
            if DEBUG:
                cv2.imshow("original", img)
                cv2.imshow("binary", bin)
                cv2.imshow("processed", edgesColor)

                cv2.waitKey(5)
                
            cv2.imshow(t + " " + str(i), out)
            

if __name__ == '__main__': #start here
    print("bruh program")
    sys.setrecursionlimit(2500) #this is needed because getTargets() uses some monster recursion to go through the contour tree
    main()
    cv2.waitKey()
    