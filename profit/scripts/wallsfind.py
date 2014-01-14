#!/usr/bin/env python

import rospy
import numpy as np
import cv2
import math
from geometry_msgs.msg import Twist



def wallsfind():
    cv2.startWindowThread()

    cv2.namedWindow('blueonly')
    cv2.namedWindow('blue')
    cv2.namedWindow('white')

    cap = cv2.VideoCapture(1)
    cap.set(3,320)
    cap.set(4,240)

    WIDTH = 320
    pixs = np.zeros(32)
    ind = 1

    rospy.init_node('findballs')
    pub = rospy.Publisher('/cmd_vel', Twist)
    msg = Twist()

    r = rospy.Rate(30)

    pixsl = 80
    anglel = 0

    while not rospy.is_shutdown():

        # this function draws dots. It's kinda neat, because dots!
        def dot(x, y):
            cv2.circle(frame, (x, y), 1, (0, 255, 0), -1)

        ret, orig = cap.read()
        frame = orig.copy()
        blur = cv2.blur(frame, (5,2))
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

        lower_blue1 = np.array([95,255*0.15,255*0.1])
        upper_blue1 = np.array([125,255*0.7,255*0.9])

        imgBlue = cv2.inRange(hsv, lower_blue1, upper_blue1)
#     src2 = cv2.inRange(hsv, lower_red2, upper_red2)
#     combined = cv2.bitwise_or(src1, src2)
#

        #frame = cv2.cornerHarris(imgBlue, 2, 3, 0.05)
        fContours = imgBlue.copy()
        contours, hierarchy = cv2.findContours(fContours, 1, 1)
#
#     for s in contours:
#       area = cv2.contourArea(s)
#       if area > 100:
#         print area
#         M = cv2.moments(s)
#         cx = int(M['m10']/M['m00'])
#         cy = int(M['m01']/M['m00'])
#         r = (area / 3.14)**0.5
#         cv2.circle(frame, (cx, cy), int(r), (0, 255, 0), 1)
#         print cx, cy
#     # M = cv2.moments(cnt)
#     # contours, hierarchy = cv2.findContours(src1,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
#     # contourImg = frame.copy()


        # sets contour to be the largest contour; must be a better way to do this :(.. CV_RETR_EXTERNAL doesn't do
        # what I thought it does. bluuuuuuuuuuuuuu
        area = 0;
        contour = ''
        for cnt in contours:
            a = cv2.contourArea(cnt)
            if a > area:
                area = a
                contour = cnt

        # draws a line right on the money. the money being the blue line. Make this code better, geez.
        if len(contour) > 1:
            line = cv2.fitLine(contour, 1, 0, 0.01, 0.01)
            #pt1 = (line[0]*10, line[1]*10)
            #pt2 = (line[2], line[3])
            m = line[1]/line[0]
            b = line[3]-m*line[2]
            # dot(0, b)
            pt1 = (0, b)
            pt2 = (WIDTH,m*WIDTH + b)
            cv2.line(frame, pt1, pt2, (0, 255, 0))

        # now lets draw TWO LINES! WOOT WOOT
        upper = []
        lower = []
        for pt in contour:
            if m*pt[0][0]+b > pt[0][1]:
                upper.append((pt[0][0], pt[0][1]))
            else:
                lower.append((pt[0][0], pt[0][1]))

        if len(upper) > 1:
            upperline = cv2.fitLine(np.array(upper), 1, 0, 0.01, 0.01)

            mu = upperline[1]/upperline[0]
            bu = upperline[3]-mu*upperline[2]
            pt1u = (0, bu)
            pt2u = (WIDTH,mu*WIDTH + bu)
            cv2.line(frame, pt1u, pt2u, (0, 0, 255))

        if len(lower) > 1:
            lowerline = cv2.fitLine(np.array(lower), 1, 0, 0.01, 0.01)
            ml = lowerline[1]/lowerline[0]
            bl = lowerline[3]-ml*lowerline[2]
            pt1l = (0, bl)
            pt2l = (WIDTH,ml*WIDTH + bl)
            cv2.line(frame, pt1l, pt2l, (0, 0, 255))

        leftSeg = bl - bu
        rightSeg = (ml*WIDTH + bl) - (mu*WIDTH + bu)

        angle = leftSeg - rightSeg
        distAway = (leftSeg + rightSeg) / 2
        # hull = cv2.convexHull(cnt)

        angleAwayDeg = angle[0]*0.4 + anglel*0.6
        anglel = angleAwayDeg
        # print angleAwayDeg

        msg.angular.z = np.clip(angleAwayDeg/100, -0.4, 0.4)

        distAwayCm = (100 - distAway[0])*0.4 + pixsl*0.6
        pixs[ind] = 100 - distAway[0]
        ind = (ind+1)%32
        print np.mean(pixs)

        msg.linear.x = np.clip((distAwayCm-77)/80, -0.12, 0.12)

        pixsl = distAwayCm

        blank = np.ones((240,320))
        cv2.line(blank, (0,30), (320,30), (0, 0, 255))
        cv2.circle(blank, (160,int(30 + distAwayCm)), 30, (0, 0, 255))
        x = math.sin(math.radians(angleAwayDeg)) * 30
        y = math.cos(math.radians(angleAwayDeg)) * 30

        cv2.line(blank, (160,int(30+distAwayCm)), (160+int(x), int(30+distAwayCm)-int(y)), (0, 0, 255))
        # cv2.rectangle(blank, (140,30+distAwayCm), (180,70+distAwayCm), (0, 0, 255))



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
        cv2.imshow('white',blank)

        pub.publish(msg)

        r.sleep()

# When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':

    try:
        wallsfind()
    except rospy.ROSInterruptException:
        pass

