#!/usr/bin/env python
import roslib; roslib.load_manifest('b2b')
import rospy
from geometry_msgs.msg import Twist, TwistStamped, PointStamped
from std_msgs.msg import Empty
from b2b.msg import IRStamped
import math
import tf

op = TwistStamped()

def cmd_vel_cb(msg):
    global op
    op.twist.linear.x = msg.linear.x
    op.twist.angular.z = msg.angular.z

def reset_odom(msg):
    global op
    op = TwistStamped()

def main():
    global op
    rospy.init_node('pretend_bot')
    r = rospy.Rate(60)
    rospy.Subscriber('/cmd_vel', Twist, cmd_vel_cb)
    rospy.Subscriber('/odom_reset', Empty, reset_odom)
    pub = rospy.Publisher('/odom_partial', TwistStamped)
    pub2 = rospy.Publisher('/ir_raw', IRStamped)

    pt = PointStamped()
    listener = tf.TransformListener()

    frame = 0
    irs = IRStamped()

    irs.l.fwd = float('inf')
    irs.l.mid = float('inf')
    irs.l.bak = float('inf')
    irs.r.fwd = float('inf')
    irs.r.mid = float('inf')
    irs.r.bak = float('inf')
    irs.fwd_l = float('inf')
    irs.fwd_r = float('inf')

    while not rospy.is_shutdown():
        op.header.stamp = rospy.Time.now()
        irs.header.stamp = rospy.Time.now()
        op.twist.angular.x += op.twist.linear.x*math.cos(op.twist.linear.z) / 120.0
        op.twist.angular.y += op.twist.linear.x*math.sin(op.twist.linear.z) / 120.0
        op.twist.linear.z += op.twist.angular.z / 60.0
        op.twist.angular.x += op.twist.linear.x*math.cos(op.twist.linear.z) / 120.0
        op.twist.angular.y += op.twist.linear.x*math.sin(op.twist.linear.z) / 120.0

        pub.publish(op)
        pub2.publish(irs)

        frame += 1

        r.sleep()


if __name__=='__main__':
    main()

