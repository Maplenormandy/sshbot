#!/usr/bin/env python
import roslib; roslib.load_manifest('profit')
import rospy
import numpy as np
import cv2
from profit.msg import Ball, BallArray
from ballseeingeye import BallSeeingEye

def main():
    cv2.startWindowThread()
    cv2.namedWindow('FullImage')
    cv2.namedWindow('Threshold')

    rospy.init_node('ballseeingeye')
    ball_pub = rospy.Publisher('/balls', BallArray)
    rate = rospy.Rate(30)

    def ballCb(ballsList):
        msg = BallArray()
        msg.header.stamp = rospy.get_rostime()
        for (x,y,r,color) in ballsList:
            ball = Ball(x,y,r,color)
            ball.x = x
            ball.y = y
            ball.r = r
            ball.color = color
            msg.balls.append(ball)

        ball_pub.publish(msg)

    bse = BallSeeingEye(ballCb=ballCb, debug=True, quickstart=True)

    while not rospy.is_shutdown():
        bse.loop()
        rate.sleep()

if __name__=='__main__':
    main()
