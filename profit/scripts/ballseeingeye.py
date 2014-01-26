# BallSeeingEye SSHBOT LongIsland Vision System
#
# COMMAND LINE:
# -d : uses debug mode with camera output
# -i : sets findBalls and findWalls to be initialize as True

from __future__ import print_function
import numpy as np
import cv2
import sys
import time

class BallSeeingEye:
    AREA_THRESHOLD = 100
    BLUR = 10
    DESIRED_WIDTH = 352
    DESIRED_HEIGHT = 288
    SLEEP = 20

    WALL_LOOKAHEAD_BOUNDARY = 10  # cap the edges of the camera
    WALL_EDGE_THRESHOLD = 10      # cap the edges of the walls

    # Colour Theshold Values (OUT OF). H(180), S(1), V(1).
    RED = [[0,   10,  .3, .95, .2, .95],
           [170, 180, .3, .73, .3, .9]]
    GREEN = [[49, 80, .17, .8, .1, .8]]
    #BLUE = [[100, 140,  .1, .9, .1, .9]]
    BLUE = [[95, 150,  0.0, 1.0, 0.4, 1.0]]
    TEAL = [[80, 95,  .1, .9, .2, .9]]
    YELLOW = [[10, 40, .3,  .9, .3, .99]]
    COLOURS = {'R': RED, 'G': GREEN, 'B': BLUE, 'Y': YELLOW, 'T': TEAL}
    LINE_COLOURS = {'Y': (0,0,255), 'B': (0,255,0), 'R': (255,255,0), 'T': (255,0,0), 'G':(0,0,255)}

    def __init__(self, ballCb=print, wallCb=print,
            camera=1, debug=False, quickstart = False):
        self.cap = cv2.VideoCapture(camera)
        print("Setting up camera feed...")
        time.sleep(1)

        self.cap.set(3,self.DESIRED_WIDTH)
        self.cap.set(4,self.DESIRED_HEIGHT)
        print("Configuring size...")
        time.sleep(1)

        self.WIDTH = int(self.cap.get(3))
        self.HEIGHT = int(self.cap.get(4))
        print("Actual Dimensions: "
          + str(self.WIDTH) + ", " + str(self.HEIGHT))

        # Callbacks for vision code
        self.ballCb = ballCb
        self.wallCb = wallCb

        self.debug = debug
        if quickstart:
            self.findBalls = True
            self.findWalls = True
        else:
            self.findBalls = False
            self.findWalls = False


    def watch(self):
        """ Parameters that turn on and off processing goes here """

        self.findBalls = True
        self.findWalls = True

    def loop(self):
        # Read Camera Properties
        ret, orig = self.cap.read()
        self.WIDTH = orig.shape[1]
        self.HEIGHT = orig.shape[0]

        if self.debug:
            print(self.WIDTH, 'x', self.HEIGHT)

        frame = orig.copy()
        post = frame.copy()


        crop = post[(self.HEIGHT/2):,:]
        blur = cv2.blur(crop, (self.BLUR, self.BLUR))


        if not blur == None:
            hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

            # Find Yellow WALLS
            wallsList = []
            forbidden = []
            yellowWalls = self.wallsFind(hsv, frame, 'Y')
            if not yellowWalls == None:
                wallsList.append(yellowWalls)

            # Find Blue WALLS
            blueWalls = self.wallsFind(hsv, frame, 'B')
            if not blueWalls == None:
                wallsList.append(blueWalls)

            # Find Red WALLS
#             redWalls = self.wallsFind(hsv, frame, 'R')
#             if not redWalls == None:
#                 wallsList.append(redWalls)
#                 forbidden.append(redWalls)

            # Find Teal WALLS
            tealWalls = self.wallsFind(hsv, frame, 'G')
            if not tealWalls == None:
                wallsList.append(tealWalls)
                forbidden.append(tealWalls)

            # Find BALLS
            ballsList = self.ballsFind(hsv, frame, forbidden)

            if self.findBalls:
                self.ballCb(ballsList)

            if self.findWalls:
                self.wallCb(wallsList)

        if self.debug:
            cv2.imshow('FullImage', frame)

        return False

    def execute(self):
        while True:
            if self.loop():
                break

            if cv2.waitKey(self.SLEEP) & 0xFF == ord('q'):
                break

        self.cap.release()
        cv2.destroyAllWindows()

    # Returns a list of balls in the format [x, y, r, colour]
    def ballsFind(self, img, frame, forbiddenZone):
        hsv = img.copy()
        thresholdRed = self.threshold('R', hsv)
        thresholdGreen = self.threshold('G', hsv)


        cv2.imshow('G', thresholdGreen)

        ballsList = []
        for thresholdImg in [(thresholdRed, 'r'), (thresholdGreen,'g')]:
            contours, hierarchyRed = cv2.findContours(thresholdImg[0].copy(), 1, 2)
            for s in contours:
                area = cv2.contourArea(s)
                if area > self.AREA_THRESHOLD:
                    M = cv2.moments(s)
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])
                    x = (cx-self.WIDTH/2.0) / self.HEIGHT
                    y = (cy*1.0) / self.HEIGHT
                    r = (area / 3.14)**0.5 / self.HEIGHT

                    # fitline - check if dot is in forbidden zone (a wall)
                    # change to function when more awake

                    validate = True

                    if len(forbiddenZone) > 0:
                        for fz in forbiddenZone:
                            m = (fz[1][1] - fz[2][1])/(fz[1][0] - fz[2][0])
                            b = fz[1][1] - m * fz[1][0]
                            topY = m * cx + b
                            m = (fz[3][1] - fz[4][1])/(fz[3][0] - fz[4][0])
                            b = fz[3][1] - m * fz[3][0]
                            botY = m * cx + b
                            if (cy+self.HEIGHT*0.5) > topY and (cy+self.HEIGHT*0.5) < botY:
                                validate = False

                    if validate:
                        ballsList.append((x, y, r, thresholdImg[1]))
                        if self.debug:
                            cv2.circle(frame,
                                (cx, cy+self.HEIGHT/2), int(r * self.HEIGHT), (0, 255, 0), 1)


        return ballsList

    def wallsFind(self, img, frame, colour='B'):
        hsv = img.copy()
        thresholdColour = self.threshold(colour, hsv)

        if self.debug:
            cv2.imshow(colour, thresholdColour)

        contours, hierarchy = cv2.findContours(thresholdColour, 1, 1)

        area = 100;
        contour = None
        for cnt in contours:
            a = cv2.contourArea(cnt)
            if a > area:
                area = a
                contour = cnt

        if not contour == None:

            # Draws the center line of the wall
            m, b = self.getLine(np.array(contour), frame, (0,0,0))

            # Pick the left and right ends of the wall
            leftMost = self.WIDTH
            rightMost = 0
            for pt in contour:
                if pt[0][0] < leftMost:
                    leftMost = pt[0][0]
                if pt[0][0] > rightMost:
                    rightMost = pt[0][0]

            # Sorts the points into upper and lower
            upper = []
            lower = []
            for pt in contour:
                if (pt[0][0] > self.WALL_LOOKAHEAD_BOUNDARY and
                        pt[0][0] < self.WIDTH - self.WALL_LOOKAHEAD_BOUNDARY and
                        pt[0][0] - leftMost > self.WALL_EDGE_THRESHOLD and
                        rightMost - pt[0][0] > self.WALL_EDGE_THRESHOLD):
                    if m*pt[0][0]+b > pt[0][1]:
                        upper.append((pt[0][0], pt[0][1]))
                    else:
                        lower.append((pt[0][0], pt[0][1]))

            mu, bu = self.getLine(np.array(upper), frame, self.LINE_COLOURS[colour])
            ml, bl = self.getLine(np.array(lower), frame, self.LINE_COLOURS[colour])

            topLeft = (np.array(leftMost), np.array(mu * leftMost + bu + self.HEIGHT/2))
            bottomLeft = (np.array(leftMost), np.array(ml * leftMost + bl + self.HEIGHT/2))
            topRight = (np.array(rightMost), np.array(mu * rightMost + bu + self.HEIGHT/2))
            bottomRight = (np.array(rightMost), np.array(ml * rightMost + bl + self.HEIGHT/2))

            # Better than a random number generator, I promise.
            confidence = max(1 - abs(mu-ml) - abs(mu)*0.5 - (300/area), 0)

            if self.debug:
                cv2.line(frame, topLeft, bottomLeft, self.LINE_COLOURS[colour])
                cv2.line(frame, topRight, bottomRight, self.LINE_COLOURS[colour])

            return [colour, topLeft, topRight, bottomRight, bottomLeft, confidence]

        return None

    ### VHFMA - Various Helping for My Amusement

    # Returns a thresholded b/w image of the color
    def threshold(self, colour, img):

        hsv = img.copy()
        if colour == 'R':
            l = self.COLOURS[colour][0]
            u = self.COLOURS[colour][1]
            lower_red1 = np.array([l[0],255*l[2],255*l[4]])
            upper_red1 = np.array([l[1],255*l[3],255*l[5]])
            lower_red2 = np.array([u[0],255*u[2],255*u[4]])
            upper_red2 = np.array([u[1],255*u[3],255*u[5]])
            src1 = cv2.inRange(hsv, lower_red1, upper_red1)
            src2 = cv2.inRange(hsv, lower_red2, upper_red2)
            return cv2.bitwise_or(src1, src2)
        else:
            l = self.COLOURS[colour][0]
            lower_thresh = np.array([l[0],255*l[2],255*l[4]])
            upper_thresh = np.array([l[1],255*l[3],255*l[5]])
            return cv2.inRange(hsv, lower_thresh, upper_thresh)

    # Returns m, b, upon taking in a numpy set of points & frame to draw on
    def getLine(self, points, frame, color=(0, 0, 255)):
        if len(points) > 0:
            upperline = cv2.fitLine(points, 1, 0, 0.01, 0.01)

            # shim to prevent divide by zero
            upperline[0] = max(upperline[0], 0.0001)
            m = (upperline[1]/upperline[0])
            b = (upperline[3]-m*upperline[2])

            if self.debug:
                pt1 = (0, b + self.HEIGHT/2)
                pt2 = (self.WIDTH,m*self.WIDTH + b + self.HEIGHT/2)
                cv2.line(frame, pt1, pt2, color)
            return m[0], b[0]
        else:
            return 0, 0

if __name__ == '__main__':
    args = sys.argv
    bse = BallSeeingEye(
        debug=("-d" in args), quickstart=("-i" in args))
    bse.execute()

