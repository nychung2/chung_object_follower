#!/usr/bin/env python3

import numpy as np
import cv2

global color, run, frame

def trackbar_hue(val):
    global thue
    thue = val
    cv2.setTrackbarPos('Hue Margin', 'filtered', thue)

def trackbar_sat(val):
    global tsat
    tsat = val
    cv2.setTrackbarPos('Saturation Margin', 'filtered', tsat)

def trackbar_value(val):
    global tval
    tval = val
    cv2.setTrackbarPos('Value Margin', 'filtered', tval)

def get_color(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDBLCLK:
        global color, run
        color = hsv_frame[y, x]
        print("RGB Color Target: %s" %(color))
        run = True
        return
    
def get_object_location(contours):
    if len(contours) != 0:
        c = max(contours, key = cv2.contourArea)
        x,y,w,h = cv2.boundingRect(c)
        return x, y, w ,h
    else:
        return None, None, None, None

def get_center(x,y,w,h):
    cx = x + w/2
    cy = y + h/2
    print("Center Coordinates of Object = (%s,%s)" %(cx, cy))
    return (cx, cy)

size = 7
tune = True
run = False
color = None
cap = cv2.VideoCapture(0)
thue = 5
tsat = 50
tval = 50
hue = 5
sat = 50
val = 50

cv2.namedWindow('camera')
cv2.namedWindow('filtered')

if tune:
    cv2.createTrackbar('hue', 'filtered', thue, 360//2, trackbar_hue)
    cv2.createTrackbar('saturation', 'filtered', tsat,255, trackbar_sat)
    cv2.createTrackbar('value', 'filtered', tval, 255, trackbar_value)

while(True):
    # Set Kernel Size
    kernel = np.ones((size,size), np.uint16)

    # Capture frame-by-frame
    ret, frame = cap.read()
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Run image processing
    if run:
        if tune:
            low_bound = np.array([color[0]-thue, color[1]-tsat, color[2]-tval])
            high_bound = np.array([color[0]+thue, color[1]+tsat, color[2]+tval])
            hsv_threshold = cv2.inRange(hsv_frame, low_bound, high_bound)
        else:
            low_bound = np.array([color[0]-hue, color[1]-sat, color[2]-val])
            high_bound = np.array([color[0]+hue, color[1]+sat, color[2]+val])
            hsv_threshold = cv2.inRange(hsv_frame, low_bound, high_bound)
        close_dialation = cv2.dilate(hsv_threshold, kernel, iterations=1)
        close_erosion = cv2.erode(close_dialation, kernel, iterations=1)
        open_erosion = cv2.erode(close_erosion, kernel, iterations=1)
        open_dialation = cv2.dilate(open_erosion, kernel, iterations=1)

        # Try find contours
        contours, hierarchy = cv2.findContours(open_dialation, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        display = frame
        if len(contours) != 0:
            x,y,w,h = get_object_location(contours)
            get_center(x,y,w,h)
            cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)
            #cv2.drawContours(frame, contours, 0, (0,255,0))  
            cv2.putText(frame, "Object", (x,y), color=(0,255,0), thickness=1, fontFace=0, fontScale=1)

    # Display the resulting frame
    cv2.imshow('camera',frame)
    if run:
        cv2.imshow('filtered',open_dialation)

    # get color
    cv2.setMouseCallback('camera', get_color)
    
    # Key Inputs
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()