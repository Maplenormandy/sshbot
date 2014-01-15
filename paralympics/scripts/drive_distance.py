#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Float32
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, TwistStamped
from dynamic_reconfigure.server import Server
from wallfollower.cfg import WallflowerConfig
import math
import numpy as np

odom = None
cmd_vel = None

def odomCb(msg):
    global cmd_vel
    odom = msg

def main():
    global odom, cmd_vel
    rospy.init_node('boxxy')

    cmd_vel = rospy.Publisher("/cmd_vel", Twist)
    rospy.Subscriber("/odom_partial", TwistStamped, odomCb)

    msg = Twist()

    dist = 0.3048*5

    tStraight = dist / 0.1

    while not rospy.is_shutdown():
        msg.linear.x = 0.1
        msg.angular.z = 0
        cmd_vel.publish(msg)
        rospy.sleep(tStraight)

        rospy.sleep(1.0)

        msg.linear.x = -0.1
        msg.angular.z = 0
        cmd_vel.publish(msg)
        rospy.sleep(tStraight)

        rospy.spin()


if __name__=='__main__':
    main()



