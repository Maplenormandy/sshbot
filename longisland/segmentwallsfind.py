import numpy as np
import cv2
import math

cap = cv2.VideoCapture(1)
cap.set(3,320)
cap.set(4,240)

WIDTH = 320

    
while(True):

    # this function draws dots. It's kinda neat, because dots!
    def dot(x, y):
       cv2.circle(frame, (x, y), 1, (0, 255, 0), -1)
       
    ret, orig = cap.read()  
    frame = orig.copy()
    blur = cv2.blur(frame, (10,5))
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
        
    lower_blue1 = np.array([98,255*0.2,255*0.3])
    upper_blue1 = np.array([120,255*0.9,255*0.9])
    
    imgBlue = cv2.inRange(hsv, lower_blue1, upper_blue1)
    fContours = imgBlue.copy()
    contours, hierarchy = cv2.findContours(fContours, 1, 1)

    print "--"
    
    # sets contour to be the largest contour; must be a better way to do this :(.. CV_RETR_EXTERNAL doesn't do
    # what I thought it does. bluuuuuuuuuuuuuu
    area = 0;
    contour = ''
    for cnt in contours:
      a = cv2.contourArea(cnt)
      if a > area:
        area = a
        contour = cnt
    
    cv2.drawContours(frame, contour, -1, (100,100,100), 1)
    
    # draws a line right on the money. the money being the blue line. Make this code better, geez.
    line = cv2.fitLine(contour, 1, 0, 0.01, 0.01)
    #pt1 = (line[0]*10, line[1]*10)
    #pt2 = (line[2], line[3])
    m = line[1]/line[0]
    b = line[3]-m*line[2]
    # dot(0, b)
    pt1 = (0, b)
    pt2 = (WIDTH,m*WIDTH + b)
    cv2.line(frame, pt1, pt2, (255, 255, 0))
    
    # increase this number if you want your code to run slower
    SEGMENTS = 10
    upper, lower = [],[]
    for i in range(SEGMENTS):
      upper.append([])
      lower.append([])
    
    def lineFetch(pts):
      line = cv2.fitLine(pts, 1, 0, 0.01, 0.01)
      m = line[1]/line[0]
      b = line[3]-m*line[2]
      return (m, b)

    # categorize the points.
    for pt in contour:
      x = pt[0][0]
      y = pt[0][1]
      if x > 10 and x < (WIDTH - 10): # filter out boundary points
        if m*x+b > y: # sort into top or bottom
          p = upper
        else: 
          p = lower 
        seg = int((float(x) / float(WIDTH)) * SEGMENTS)
        p[seg].append((x,y))
    #print len(upper[0]), len(upper[1]), len(upper[2]), len(upper[3])
    
    i = 0 # start analyzing at segment i
    contSeg = 1
    wallCounts = [] # (mu, bu, ml, bl, startx, endx),(.. etc
    startx = 0
    
    uppercheck = upper[0][:]
    lowercheck = lower[0][:]
    print uppercheck
    while i < SEGMENTS:
      
      if contSeg > 1:
        err = 0
        for pt in upper[i]:
          err += (mu*pt[0]+bu - pt[1])**2
        for pt in lower[i]:
          err += (ml*pt[0]+bl - pt[1])**2
        if err < 300: # change this number if you want a fun time
          uppercheck.extend(upper[i][:])
          lowercheck.extend(lower[i][:])
        else:
          endx = startx + (WIDTH/SEGMENTS) * (contSeg-1)
          wallCounts.append((mu, bu, ml, bl, startx, endx))
          uppercheck = upper[i][:]
          lowercheck = lower[i][:]
          startx = endx
          contSeg = 1
        
      if contSeg > 0:
        if len(uppercheck) > 1:
          mu, bu = lineFetch(np.array(uppercheck))
        else:
          contSeg = 0
        if len(lowercheck) > 1:      
          ml, bl = lineFetch(np.array(lowercheck))
        else: 
          contSeg = 0
      
      contSeg += 1
      i += 1
    
    endx = startx + (WIDTH/SEGMENTS) * contSeg
    wallCounts.append((mu, bu, ml, bl, startx, endx))
    
    # draw lines
    for i in range(len(wallCounts)):
      mu, bu, ml, bl, startx, endx = wallCounts[i]
      pt1u = (startx, mu*startx + bu)
      pt2u = (endx,mu*endx + bu)
      cv2.line(frame, pt1u, pt2u, (0, 255*(i%2), 255*-(i%2)))
      pt1l = (startx, ml*startx + bl)
      pt2l = (endx,ml*endx + bl)
      cv2.line(frame, pt1l, pt2l, (0, 255*-(i%2), 255*(i%2)))
        
#     angle = leftSeg - rightSeg
#     distAway = (leftSeg + rightSeg) / 2
         
#     def pixelsToCMDepth(p):
#       # shibe i'm too lazy to figure this out
#       return 100 - p
#     def pixelsToAngle(p):
#       # again, shibe is allowing me to be lazy
#       return p
#       
#     angleAwayDeg = pixelsToAngle(angle)
#     distAwayCm = pixelsToCMDepth(distAway) #- angleAwayDeg/10 #fudge factor, for fun
#     
#     # Lol, this draws a robot.
#     blank = np.ones((240,320))
#     cv2.line(blank, (0,30), (320,30), (0, 0, 255))
#     cv2.circle(blank, (160,30 + distAwayCm), 30, (0, 0, 255))
#     x, y = math.sin(math.radians(angleAwayDeg)) * 30, math.cos(math.radians(angleAwayDeg)) * 30
#     
#     cv2.line(blank, (160,30+distAwayCm), (160+int(x), 30+distAwayCm-int(y)), (0, 0, 255))
#     # cv2.rectangle(blank, (140,30+distAwayCm), (180,70+distAwayCm), (0, 0, 255))
#     

    
      # 15cm = 41
      # 20cm = 32
      # 24cm = 26
#       print cnt
#       i = 0
#       for point in cnt:
#         i += 1
#         print point
#         cv2.circle(frame, (point[0][0], point[0][1]), 1, (0, 255, 0), -1)
#         if i > 100:
#           break
          
#    cv2.drawContours(frame, cnt, -1, (0,255,0), 3)
#     # cv2.circle(frame, (0, 0), 10, (0, 255, 0), -1)

#    cv2.imshow('original', orig)
    cv2.imshow('blueonly', imgBlue)
    cv2.imshow('blue', frame)
#    cv2.imshow('white',blank)
    
    if cv2.waitKey(200) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()