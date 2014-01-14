#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

def irCb(msg):
    a = msg.ranges[0]
    b = msg.ranges[1]

    sym = a+b
    anti = a-b



def wallflower():
    rospy.init_node('wallflower')
    rospy.Subscriber("/ir_data", LaserScan, irCb)

    global pub = rospy.Publisher("/cmd_vel", Twist)

    rospy.spin()

if __name__ == '__main__':
    try:
        wallflower()
    except rospy.ROSInterruptException:
        pass
