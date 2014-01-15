#!/usr/bin/env python
import roslib; roslib.load_manifest('wallfollower')
import rospy
from std_msgs.msg import String, Float32
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from dynamic_reconfigure.server import Server
from wallfollower.cfg import WallflowerConfig
import math
import numpy as np

cmd_vel = None
bi = 0
bl = 0
bdl = 0

fr = 0.3
frl = 0.3

closeenuf = True

def frontCb(msg):
    global fr, frl
    if not math.isinf(msg.data):
        fr = msg.data * 0.1 + frl * 0.9
        frl = fr

def cfgCb(config, level):
    global cfg
    cfg = config
    return config

def irCb(msg):
    global cmd_vel, cfg
    global bl, bi, bdl
    global fr
    global closeenuf

    vel = Twist()

    if not closeenuf or (msg.ranges[2] > 0.29):
        if msg.ranges[1] < 0.16:
            vel.linear.x = 0.05
            vel.angular.z = 0.15
            closeenuf = msg.ranges[2] < 0.20
            rospy.loginfo('scary!')
        else:
            vel.linear.x = 0.01
            vel.angular.z = 0.21
            closeenuf = msg.ranges[2] < 0.20
            rospy.loginfo('holy shit')
    else:
        a = np.clip(msg.ranges[0], 0.05, 0.65)
        b = np.clip(msg.ranges[1], 0.05, 0.65)
        c = np.clip(msg.ranges[2], 0.05, 0.65)

        b = c - 0.22

        b = 0.1*b + 0.9*bl

        bi += b*msg.scan_time
        bi = np.clip(bi, -1.0, 1.0)
        bd = ((b-bl)/msg.scan_time * 0.1) + bdl * 0.9
        bdl = bd
        bl = b

        if b < 0:
            vel.linear.x = 0.09 + b
        else:
            vel.linear.x = np.clip(0.09 - b, 0, 0.09)

        cmd = b*cfg.kp \
                + bi*cfg.ki \
                + bd*cfg.kd

        vel.angular.z = np.clip(cmd, -0.8, 0.8)

    cmd_vel.publish(vel)

def wallflower():
    global cmd_vel

    rospy.init_node('wallflower')
    srv = Server(WallflowerConfig, cfgCb)
    cmd_vel = rospy.Publisher("/cmd_vel", Twist)
    rospy.Subscriber("/ir_data", LaserScan, irCb)
    rospy.Subscriber("/front_ir", Float32, frontCb)

    rospy.spin()

if __name__ == '__main__':
    try:
        wallflower()
    except rospy.ROSInterruptException:
        pass
