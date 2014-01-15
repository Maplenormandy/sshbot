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

class DriveState:
    def __init__(self):
        self.tdist = 0.3048
        self.tangle = math.pi/2

        self.state = 'init'
        self.side = 0

        self.unit = np.array([1, 0])
        self.pos0 = np.array([0, 0])
        self.th0 = 0

    def odomCb(self, msg):
        global cmd_vel
        vel = Twist()

        msg.twist.linear.z = msg.twist.linear.z / 1.05

        if self.state == 'init':
            vel.linear.x = 0
            vel.angular.z = 0

            self.th0 = msg.twist.linear.z
            self.pos0 = np.array([msg.twist.angular.x, msg.twist.angular.y])
            self.unit = np.array([math.cos(self.th0), math.sin(self.th0)])

            self.state = 'straight'

        if self.state == 'straight':
            pos = np.array([msg.twist.angular.x, msg.twist.angular.y])
            s = (pos - self.pos0)

            dist = np.sum(s * self.unit)

            if abs(self.tdist - dist) < 0.01:
                self.side += 1
                if self.side == 4:
                    rospy.loginfo('done')
                    self.state = 'done'
                else:
                    self.state = 'turn'
                    rospy.loginfo('turn')
                    self.th0 = msg.twist.linear.z

                vel.linear.x = 0
                vel.angular.z = 0
            else:
                vel.linear.x = np.clip((self.tdist-dist)*3, -0.1, 0.1)
                vel.angular.z = np.clip((self.th0 - msg.twist.linear.z) / 40, -0.05, 0.05)

        elif self.state == 'turn':
            t = self.tangle - (msg.twist.linear.z - self.th0)

            if abs(t) < 0.01:
                self.state = 'straight'
                rospy.loginfo('straight')
                vel.linear.x = 0
                vel.angular.z = 0
                self.th0 = msg.twist.linear.z
                self.pos0 = np.array([msg.twist.angular.x, msg.twist.angular.y])
                self.unit = np.array([math.cos(self.th0), math.sin(self.th0)])
            else:
                vel.linear.x = 0
                vel.angular.z = np.clip(t, -0.4, 0.4)
        else:
            vel.linear.x = 0
            vel.angular.z = 0

        cmd_vel.publish(vel)


def main():
    global odom, cmd_vel
    rospy.init_node('boxxy')

    s = DriveState()

    cmd_vel = rospy.Publisher("/cmd_vel", Twist)
    rospy.Subscriber("/odom_partial", TwistStamped, s.odomCb)

    rospy.spin()

if __name__=='__main__':
    main()



