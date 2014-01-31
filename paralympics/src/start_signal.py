#!/usr/bin/env python
import roslib; roslib.load_manifest('paralympics')
import rospy
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Time

def main():
    rospy.init_node('start_signal')
    pub = rospy.Publisher('/start', Time)
    pub.publish(rospy.Time.now())

if __name__=='__main__':
    main()
