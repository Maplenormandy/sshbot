#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>

ros::Publisher* p_odom_pub;
tf::TransformBroadcaster* p_odom_broadcaster;

void futzOdom(const geometry_msgs::TwistStamped& msg)
{

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat =
        tf::createQuaternionMsgFromYaw(msg.twist.linear.z);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = msg.header.stamp;
    odom_trans.header.seq = msg.header.seq;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    // Get transformation from odom coordinates. Note that the odom_partial
    // message is strange.
    odom_trans.transform.translation.x = msg.twist.angular.x;
    odom_trans.transform.translation.y = msg.twist.angular.y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    p_odom_broadcaster->sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = msg.header.stamp;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = msg.twist.angular.x;
    odom.pose.pose.position.y = msg.twist.angular.y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = msg.twist.linear.x;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = msg.twist.angular.z;

    //publish the message
    p_odom_pub->publish(odom);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "msg_parser");

    ros::NodeHandle n;
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    ros::Subscriber odom_read = n.subscribe("odom_partial", 1000, futzOdom);
    tf::TransformBroadcaster odom_broadcaster;

    p_odom_pub = &odom_pub;
    p_odom_broadcaster = &odom_broadcaster;

    ros::spin();
    ros::spin();
}

