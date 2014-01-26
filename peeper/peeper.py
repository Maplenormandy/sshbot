# PEEPER SSHBOT LongIsland Vision System
#
# COMMAND LINE:
# -d : uses debug mode with camera output

from __future__ import print_function
import numpy as np
import cv2
import sys
import time

class peeper:
    BLUR = 30
    SLEEP = 20
    DESIRED_WIDTH = 352
    DESIRED_HEIGHT = 288
    
    RED = [[0,   10,  .3, .95, .2, .95],
           [170, 180, .3, .73, .3, .9]]
    GREEN = [[49, 80, .17, .8, .1, .8]]    
    
    EVALPOINT = (100,100)
    
    def __init__(self, colourCall=print, camera=1, debug=False):
        self.cap = cv2.VideoCapture(camera)
        print("Setting up camera feed...")
        time.sleep(1)
        
        self.cap.set(3,self.DESIRED_WIDTH)
        self.cap.set(4,self.DESIRED_HEIGHT)
        
        # Callbacks for vision code
        self.colourCall = colourCall
        self.debug = debug

    def loop(self):
        # Read Camera Properties
        ret, orig = self.cap.read()
        frame = orig.copy()
        post = frame.copy()
        
        blur = cv2.blur(post, (self.BLUR, self.BLUR))
        
        if not blur == None:
            hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
            cv2.circle(frame, self.EVALPOINT, 1, (0, 255, 0), 1)
            cv2.circle(blur, self.EVALPOINT, 1, (0, 255, 0), 1)
            
            h,s,v = hsv[self.EVALPOINT[0], self.EVALPOINT[1]]
            print(h)
            
            if self.debug:
                cv2.imshow('FullImage', blur)
            
    def execute(self):
        while True:
            if self.loop():
                break

            if cv2.waitKey(self.SLEEP) & 0xFF == ord('q'):
                break

        self.cap.release()
        cv2.destroyAllWindows()
        
if __name__ == '__main__':
    args = sys.argv
    needColourNow = peeper(
        debug=("-d" in args))
    needColourNow.execute()

