from ctypes.wintypes import RGB
import cv2 
import numpy as np

videoIn = cv2.VideoCapture(0) #change to access front camera feed

openingKernel= np.ones((5,5), np.uint8)
openingHandleKernel = np.ones((2,2), np.uint8)
frameCounter = 0

while(True):
    frameCounter = frameCounter + 1

    _, rawFrame = videoIn.read() # gets a frame
    frame = rawFrame # make a duplicate

    #raw
    #cv2.imshow("FrameIn", frame)


    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    lowerBound = np.array([0, 30, 140])
    upperBound = np.array([130, 130, 210])

    #apply color filter
    filteredImage = cv2.inRange(frame, lowerBound, upperBound)
    cv2.imshow("Filtered Image", filteredImage)

    #reduce noise
    openedImage = cv2.morphologyEx(filteredImage, cv2.MORPH_OPEN, openingKernel)
    #cv2.imshow("Opened Image", openedImage)

    #finds bin contour
    contours, _ = cv2.findContours(openedImage, 1, 2)
    

    #only process if there is a contour
    if(len(contours) > 0):

        #get the larget contour
        maxArea = np.array([-1, -1])
        counter = 0
        for contour in contours:
            if(cv2.contourArea(contour) > maxArea[0]):
                maxArea[0] = cv2.contourArea(contour) 
                maxArea[1] = counter

            counter = counter + 1

        binContour = contours[maxArea[1]]
        print(cv2.contourArea(binContour))
        
        # bin contour must be of size - no dots
        if(cv2.contourArea(binContour) > 50):
            binBoundingRect = cv2.minAreaRect(binContour)
            points = cv2.boxPoints(binBoundingRect)
            points = np.int0(points)
            cv2.drawContours(frame, [points], 0, (0,255,255), 2)

            #show frame with a drawn target
            cv2.imshow("Detection Frame", frame)

            binLocation  = binBoundingRect[0]# the location of the bin
            binArea = maxArea[0] # the are of the bin - use to derive distance?
            
            print(binLocation)

            #get the flattened bound rectangle
            x, y, w, h = cv2.boundingRect(binContour)
            binImage = rawFrame[y:y+h,x:x+w] #image with only the bin
            cv2.imshow("Bin View", binImage)

            #filter for the handle by color
            lowerHandleBound = np.array([100, 0, 0])
            upperHandleBound = np.array([255, 90, 255])
            handleColorFilterredImage = cv2.inRange(binImage, lowerHandleBound, upperHandleBound)
            cv2.imshow("Handle Filter", handleColorFilterredImage)
            
            #reduce noise in the handle mask
            handleOpenedImage = cv2.morphologyEx(handleColorFilterredImage, cv2.MORPH_OPEN, openingHandleKernel)
            cv2.imshow("Opened Handle", handleOpenedImage)

            #find handle contours
            handleContours, _ = cv2.findContours(handleOpenedImage, 1, 2)

            #only attempt to process the handle contours if there are any detected
            if(len(handleContours) > 0):
                maxHandleArea = np.array([-1,-1])
                handleCounter = 0
            
                #indentify the handle contour by size
                for contour in handleContours:
                    if (cv2.contourArea(contour) > maxHandleArea[0]):
                        maxHandleArea[0] = cv2.contourArea(contour)
                        maxHandleArea[1] = handleCounter
                    
                    handleCounter = handleCounter + 1

                handleContour = handleContours[maxHandleArea[1]]
                #make sure the contour is not a dot...
                if(cv2.contourArea(handleContour) > 50):
                    hx,hy, hw, hh = cv2.boundingRect(handleContour)
                    cv2.rectangle(frame, (x+hx,y+hy), (x+hx+hw, y+hy+hh), (0, 255, 0), 2)

                cv2.imshow("Final Frame", frame)

# clean up
videoIn.release()
cv2.destroyAllWindows()
