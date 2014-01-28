#!/usr/bin/env python
import roslib; roslib.load_manifest('profit')
import rospy
import numpy as np
import cv2
from profit.msg import BallArray, Wall, Ball
from geometry_msgs.msg import Point, PoseArray, Pose
from ballseeingeye import BallSeeingEye

def main():

    rospy.init_node('ballseeingeye')
    ball_pub = rospy.Publisher('/profit/balls', BallArray)
    balldeb_pub = rospy.Publisher('/profit/balls_debug', PoseArray)
    silo_wall_pub = rospy.Publisher('/profit/silo_wall_raw', Wall)
    reactor_wall_pub = rospy.Publisher('/profit/reactor_wall_raw', Wall)
    rate = rospy.Rate(16)

    debug=True

    def ballCb(ballsList):
        msgDeb = None

        if debug:
            msgDeb = PoseArray()
            msgDeb.header.stamp = rospy.get_rostime()
            msgDeb.header.frame_id = 'camera_link'

        msg = BallArray()
        msg.header.stamp = rospy.get_rostime()
        msg.header.frame_id = 'camera_link'
        for (x,y,r,color) in ballsList:
            ball_pos = Point()
            ball_pos.x = -.0211*(y-r)/(-.427*(y-r)+.0723)
            ball_pos.y = -.251*x/(.937*(y-r)+.0723)
            ball = Ball(ball_pos,color)
            msg.balls.append(ball)

            if debug:
                pose = Pose()
                pose.position = ball_pos
                pose.orientation.w = 1.0
                msgDeb.poses.append(pose)


        if debug:
            balldeb_pub.publish(msgDeb)
        ball_pub.publish(msg)

    def wallCb(wallsList):
        for wall in wallsList:
            # I found a teal(reactor) wall
            if wall[0] == 'G':
                msg = Wall()
                msg.a.x = (wall[1][0]-bse.WIDTH/2.0)/1.0/bse.HEIGHT
                msg.a.y = wall[1][1]/1.0/bse.HEIGHT
                msg.b.x = (wall[2][0]-bse.WIDTH/2.0)/1.0/bse.HEIGHT
                msg.b.y = wall[2][1]/1.0/bse.HEIGHT
                msg.c.x = (wall[3][0]-bse.WIDTH/2.0)/1.0/bse.HEIGHT
                msg.c.y = wall[3][1]/1.0/bse.HEIGHT
                msg.d.x = (wall[4][0]-bse.WIDTH/2.0)/1.0/bse.HEIGHT
                msg.d.y = wall[4][1]/1.0/bse.HEIGHT

                reactor_wall_pub.publish(msg)
            elif wall[0] == 'B':
                msg = Wall()
                msg.a.x = (wall[1][0]-bse.WIDTH/2.0)/1.0/bse.HEIGHT
                msg.a.y = wall[1][1]/1.0/bse.HEIGHT
                msg.b.x = (wall[2][0]-bse.WIDTH/2.0)/1.0/bse.HEIGHT
                msg.b.y = wall[2][1]/1.0/bse.HEIGHT
                msg.c.x = (wall[3][0]-bse.WIDTH/2.0)/1.0/bse.HEIGHT
                msg.c.y = wall[3][1]/1.0/bse.HEIGHT
                msg.d.x = (wall[4][0]-bse.WIDTH/2.0)/1.0/bse.HEIGHT
                msg.d.y = wall[4][1]/1.0/bse.HEIGHT

                silo_wall_pub.publish(msg)




    bse = BallSeeingEye(ballCb=ballCb, wallCb=wallCb,
            debug=False, quickstart=True)

    while not rospy.is_shutdown():
        bse.loop()
        rate.sleep()

if __name__=='__main__':
    main()
