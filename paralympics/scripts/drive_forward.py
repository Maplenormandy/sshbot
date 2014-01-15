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

done = False

target = 0.3048*5/2

def odomCb(msg):
    global cmd_vel, target, done
    vel = Twist()

    vel.linear.x = np.clip(target-msg.twist.angular.x, -0.1, 0.1)

    if abs(vel.linear.x) < 0.01:
        if target > 0:
            target = 0
        else:
            done = True

    if not done:
        print vel
        cmd_vel.publish(vel)
    else:
        vel.linear.x = 0
        cmd_vel.publish(vel)

def main():
    global odom, cmd_vel
    rospy.init_node('boxxy')

    cmd_vel = rospy.Publisher("/cmd_vel", Twist)
    rospy.Subscriber("/odom_partial", TwistStamped, odomCb)

    rospy.spin()


if __name__=='__main__':
    main()



