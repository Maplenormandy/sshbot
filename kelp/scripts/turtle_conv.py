import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist, TwistStamped

odom_partial = None
turtle_cmd_vel = None

def futzOdom(msg):
    global odom_partial
    odom = TwistStamped()
    odom.twist.linear.x = msg.linear_velocity / 5.0
    odom.twist.linear.z = msg.theta
    odom.twist.angular.x = msg.x / 5.0
    odom.twist.angular.y = msg.y / 5.0
    odom.twist.angular.z = msg.angular_velocity / 5.0
    odom_partial.publish(odom)

def futzVel(msg):
    global turtle_cmd_vel
    msg.linear.x *= 5
    msg.angular.z
    turtle_cmd_vel.publish(msg)

def main():
    global turtle_cmd_vel, odom_partial
    rospy.init_node('turtle_conv')
    odom_partial = rospy.Publisher("/odom_partial", TwistStamped)
    turtle_cmd_vel = rospy.Publisher("/turtle1/cmd_vel", Twist)
    rospy.Subscriber("/cmd_vel", Twist, futzVel)
    rospy.Subscriber("/turtle1/pose", Pose, futzOdom)
    rospy.spin()


if __name__=='__main__':
    main()
