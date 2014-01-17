# BallSeeingEye SSHBOT LongIsland Vision System
#
# COMMAND LINE:
# -d : uses debug mode with camera output
# -i : sets findBalls and findWalls to be initialize as True

import numpy as np
import cv2
import sys

class BallSeeingEye:
    AREA_THRESHOLD = 100
    BLUR = 10
    WIDTH = 320
    HEIGHT = 240
    SLEEP = 20
    CAMERA = 1
    
    # Colour Theshold Values (MAX). H(180), S(1), V(1). 
    RED = [[0,   10,  .3, .95, .2, .95],
           [170, 180, .3, .73, .3, .9]]
    GREEN = [[50, 71, .17, .8, .1, .8]]
    
    def __init__(self, debug=False, quickstart = False):
        cap = cv2.VideoCapture(self.CAMERA)
        cap.set(3,self.WIDTH)
        cap.set(4,self.HEIGHT)
        
        self.debug = debug
        if quickstart:
            self.findBalls = True
            self.findWalls = True
        
        self.execute(cap)
        cap.release()
        cv2.destroyAllWindows()
    
    def watch(self):
        """ Parameters that turn on and off processing goes here """
        
        self.findBalls = True
        self.findWalls = True
    
    def execute(self, cap):
        while(True):
            ret, orig = cap.read()  
            frame = orig.copy()
            
            crop = frame.copy()[(self.HEIGHT/2):,:]
            blur = cv2.blur(crop, (self.BLUR,self.BLUR))
            hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
            
            if self.findBalls:
              ballsList = self.ballsFind(hsv, frame)
              
              # Final list of Balls as a list of [x, y, r, colour]
              print ballsList
            
            if self.debug:
                cv2.imshow('FullImage', frame)
            
            if cv2.waitKey(self.SLEEP) & 0xFF == ord('q'):
                break
            
    def ballsFind(self, img, frame):
        hsv = img.copy()
        thresholdRed = self.threshold('R', hsv)
        thresholdGreen = self.threshold('G', hsv)
        
        ballsList = []
        for thresholdImg in [(thresholdRed, 'r'), (thresholdGreen,'g')]:
            contours, hierarchyRed = cv2.findContours(thresholdImg[0].copy(), 1, 2)
            for s in contours:
                area = cv2.contourArea(s)
                if area > self.AREA_THRESHOLD:
                    M = cv2.moments(s)
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])
                    r = (area / 3.14)**0.5
                    ballsList.append([cx, cy, r, thresholdImg[1]])
                    
                    if self.debug:
                        cv2.circle(frame, (cx, cy+self.HEIGHT/2), int(r), (0, 255, 0), 1) 
        if self.debug:
            cv2.imshow('Threshold', thresholdRed)
                   
        return ballsList
            
    def threshold(self, colour, img):
        hsv = img.copy()
        if colour == 'R':
            l = self.RED[0]
            u = self.RED[1]
            lower_red1 = np.array([l[0],255*l[2],255*l[4]])
            upper_red1 = np.array([l[1],255*l[3],255*l[5]])
            lower_red2 = np.array([u[0],255*u[2],255*u[4]])
            upper_red2 = np.array([u[1],255*u[3],255*u[5]])
            src1 = cv2.inRange(hsv, lower_red1, upper_red1)
            src2 = cv2.inRange(hsv, lower_red2, upper_red2)          
            return cv2.bitwise_or(src1, src2)
            
        if colour == 'G':
            l = self.GREEN[0]
            lower_green = np.array([l[0],255*l[2],255*l[4]])
            upper_green = np.array([l[1],255*l[3],255*l[5]])
            return cv2.inRange(hsv, lower_green, upper_green)
        
if __name__ == '__main__':
    args = sys.argv
    bse = BallSeeingEye(
        debug=("-d" in args), quickstart=("-i" in args))
  