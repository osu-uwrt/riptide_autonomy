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
def numCorners(contour):
    arclength = cv2.arcLength(contour, True)
    corners = cv2.approxPolyDP(contour, 0.02 * arclength, True)
    corners = np.insert(corners, len(corners), corners[0], 0) #add first corner so that it is processed with the last as a segment
    # count = 0
        
    longSegments = []
    for i in range(1, len(corners)):
        prev = corners[i-1][0]
        curr = corners[i][0]
        if dist(prev, curr) > 250:
            longSegments.append([prev, curr])
            
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
    

# def specialThreshold(img, lowerH, upperH):
#     for row in img:
#         for col in row: #col is [h, s, v]
#             if col[2] > lowerH and col[2] < upperH:
#                 col[0] = 255
#                 col[1] = 255
    
#     return img


def isPotentialTarget(contour):
    (x, y), (w, h), a = cv2.minAreaRect(contour)
                
    # if x + (w/2) > 800 and y + (h/2) > 600 and x + (w/2) < 900:
    #     print("{}, {}: area: {}".format(x+(w/2), y+(h/2), w*h))
    
    #first test: is the object big enough?
    if w * h > 5000:
        #second test: is the object smooth enough?
        #a "smooth" object's convex hull length will be roughly equal to the length of the actual contour.
        hull = cv2.convexHull(contour)
        contourLength = cv2.arcLength(contour, True)
        hullLength = cv2.arcLength(hull, True)
        lengthRatio = contourLength / hullLength
        corners = numCorners(contour)
        # print("{}, {}: corners: {}, area: {}, length ratio: {}, aspect ratio: {}".format(x + (w/2), y + (h/2), corners, w*h, lengthRatio, w/h))
        if abs(1 - lengthRatio) < 0.2:
            #third test: is the aspect ratio good?
            if abs(1 - (w / h)) < 0.3:
                #fourth test: does it have the proper number of sides? TODO: figure out if this test is reliable
                # corners = numCorners(contour)
                # if corners == 10 or corners == 8 or corners == 4:
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


def main():
    for t in ["gman", "bootlegger"]:
        for i in range(1, 6):
            name = "testimages/" + t + "_" + str(i) + ".png"
            # name = "testimages/bootlegger_4.png"
            img = cv2.imread(name)
            out = img.copy()
            
            blurred = cv2.blur(img, (5, 5)) #needed to get rid of any grainy noise that might exist before edge detection
            edges = cv2.Canny(blurred, 15, 160)
            edges = cv2.dilate(edges, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
            edgesColor = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
            contours, hierarchy = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
            
            for contour in contours:
                x, y, w, h = cv2.boundingRect(contour)
                cv2.circle(edgesColor, (int(x + (w/2)), int(y + (h/2))), 2, (0, 255, 0), 2)
                
                corns = numCorners(contour)
                cv2.putText(edgesColor, str(corns), (int(x + (w/2)), int(y + (h/2))), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0))
            
            if len(contours) > 0:
                hierarchy = hierarchy[0]
                                
                n = 0
                approvedContours = []
                approvedContours = getTargets(contours, hierarchy)
                                
                for contour in approvedContours:
                    #image markup stuff
                    x, y, w, h = cv2.boundingRect(contour)
                    n = n + 1
                    
                    #tmp TODO: remove
                    arclength = cv2.arcLength(contour, True)
                    segments = cv2.approxPolyDP(contour, 0.02 * arclength, True)
                    for segment in segments:
                        cv2.circle(out, (segment[0][0], segment[0][1]), 2, (255, 0, 0), 2)
                    
                    # corners = numCorners(contour)
                    # print("{}: {}".format(n, corners))
                                                                                    
                    cv2.drawContours(edgesColor, contour, -1, (0, 255, 0), 2)
                    cenX = x + (w/2)
                    cenY = y + (h/2)
                    cv2.circle(out, (int(cenX), int(cenY)), 1, (0, 0, 255), 2)
                    cv2.putText(out, str(n), (int(cenX), int(cenY)), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0))
        
            #for debug stuff
            if DEBUG:
                cv2.imshow("original", img)
                cv2.imshow("processed", edgesColor)
                
            cv2.imshow(name, out)
            cv2.waitKey()
            

if __name__ == '__main__': #start here
    sys.setrecursionlimit(2500) #this is needed because getTargets() uses some monster recursion to go through the contour tree
    main()
    cv2.waitKey()
    