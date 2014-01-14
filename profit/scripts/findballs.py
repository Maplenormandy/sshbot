#!/usr/bin/env python

import rospy
import numpy as np
import cv2
from geometry_msgs.msg import Twist

def ballfinder():
    cv2.startWindowThread()
    cv2.namedWindow('frame3')
    cv2.namedWindow('thresh')
    cap = cv2.VideoCapture(1)
    # 352 x 288 is the best reliable 16x9 resolutions
    width = 352
    cap.set(3,width)
    cap.set(4,288)

    pub = rospy.Publisher('/cmd_vel', Twist)
    rospy.init_node('findballs')
    msg = Twist()

    # State variables for low pass filter
    lastCx = width/2
    lastR = 100

    rate = rospy.Rate(30)

    while not rospy.is_shutdown():

        ret, orig = cap.read()
        frame = orig.copy()
        blur = cv2.blur(frame, (12,12))
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

        lower_red1 = np.array([0,255*0.5,255*0.6])
        upper_red1 = np.array([10,255*0.9,255*0.90])

        lower_red2 = np.array([170,255*0.50,255*0.50])
        upper_red2 = np.array([180,255*0.7,255*0.95])

        src1 = cv2.inRange(hsv, lower_red1, upper_red1)
        src2 = cv2.inRange(hsv, lower_red2, upper_red2)
        combined = cv2.bitwise_or(src1, src2)

        lower_red2 = np.array([50,255*0.17,255*0.10])
        upper_red2 = np.array([71,255*0.80,255*0.80])

        combined = cv2.inRange(hsv, lower_red2, upper_red2)

        contours,hierarchy = cv2.findContours(combined.copy(), 1, 2)

        # Used to decide when to look for a ball
        maxGood = -1
        maxCx = -1
        maxR = -1

        for s in contours:
            area = cv2.contourArea(s)
            if area > 20:
                M = cv2.moments(s)
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                r = (area / 3.14)**0.5
                cv2.circle(frame, (cx, cy), int(r), (0, 255, 0), 1)
                # Track balls that are lower and closer to the center of FOV
                good = cy+abs(cx-width/2)*0.2+area*0.3
                if good > maxGood:
                    maxGood = good
                    maxCx = cx
                    maxR = r

        if maxGood > 0:
            # Lowpass filter
            maxCx = 0.7*maxCx + 0.3*lastCx
            maxR = 0.3*maxR + 0.7*lastR

            lastCx = maxCx
            lastR = maxR

            # Quadratic velocity function, slow when far and close
            # msg.linear.x = msg.linear.x*0.7 + (2.0 / (maxR + 3))*0.3
            # Proportional angular velocity control. Consider adding PID
            msg.angular.z = ((width/2 - maxCx)*0.004)
        else:
            # msg.linear.x = msg.linear.x*0.7
            msg.angular.z = msg.angular.z*0.3

        pub.publish(msg)

        cv2.imshow('frame3', frame)
        cv2.imshow('thresh', combined)

        rate.sleep()

# When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':

    try:
        ballfinder()
    except rospy.ROSInterruptException:
        pass

