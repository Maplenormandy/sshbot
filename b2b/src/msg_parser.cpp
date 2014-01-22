#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/TwistStamped.h>

ros::Publisher* p_odom_pub;
tf::TransformBroadcaster* p_odom_broadcaster;
ros::Publisher* p_scan_pub;
tf::TransformBroadcaster* p_scan_broadcaster;
double dist_to_ir_center = .1;//TODO

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

void irToLaser(const IRStamped& msg)
{
    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion lquat =
        tf::createQuaternionMsgFromYaw(msg.twist.linear.z);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped lscan_trans;
    lscan_trans.header.stamp = msg.header.stamp;
    lscan_trans.header.seq = msg.header.seq;
    lscan_trans.header.frame_id = "lscan";
    lscan_trans.child_frame_id = "base_link";

    // Get transformation from odom coordinates. Note that the odom_partial
    // message is strange.
    lscan_trans.transform.translation.x = -.05; //TODO
    lscan_trans.transform.translation.y = 0.0; //TODO
    lscan_trans.transform.translation.z = 0.0;
    lscan_trans.transform.rotation = lquat;

    //send the transform
    p_scan_broadcaster->sendTransform(lscan_trans);

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion rquat =
        tf::createQuaternionMsgFromYaw(msg.twist.linear.z);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped rscan_trans;
    rscan_trans.header.stamp = msg.header.stamp;
    rscan_trans.header.seq = msg.header.seq;
    rscan_trans.header.frame_id = "rscan";
    rscan_trans.child_frame_id = "base_link";

    // Get transformation from odom coordinates. Note that the odom_partial
    // message is strange.
    rscan_trans.transform.translation.x = 0.05; //TODO
    rscan_trans.transform.translation.y = 0.0; //TODO
    rscan_trans.transform.translation.z = 0.0;
    rscan_trans.transform.rotation = lquat;

    //send the transform
    p_scan_broadcaster->sendTransform(rscan_trans);


    sensor_msgs::LaserScan lscan;
    lscan.header.stamp = msg.header.stamp;
    lscan.header.frame_id = "lscan";
    lscan.angle_min = 0.7854;//TODO
    lscan.angle_max = 2.3562;//TODO
    lscan.angle_increment = 0.7854;//TODO
    lscan.time_increment = 0;
    lscan.scan_time = .045;
    lscan.range_min = dist_to_ir_center + .1;
    lscan.range_max = dist_to_ir_center + .8;

    lscan.ranges.resize(3);
    lscan.intensities.resize(3);
    lscan.ranges[0] = msg.l.fwd;
    lscan.ranges[1] = msg.l.mid;
    lscan.ranges[2] = msg.l.bak;
    //publish the message
    p_scan_pub.publish(lscan);
    
    sensor_msgs::LaserScan rscan;
    rscan.header.stamp = msg.header.stamp;
    rscan.header.frame_id = "rscan";
    rscan.angle_min = -0.7854;//TODO
    rscan.angle_max = -2.3562;//TODO
    rscan.angle_increment = -0.7854;//TODO
    rscan.time_increment = 0;
    rscan.scan_time = .045;
    rscan.range_min = dist_to_ir_center + .1;
    rscan.range_max = dist_to_ir_center + .8;
    
    rscan.ranges.resize(3);
    rscan.intensities.resize(3);
    rscan.ranges[0] = msg.r.fwd;
    rscan.ranges[1] = msg.r.mid;
    rscan.ranges[2] = msg.r.bak;
    //publish the message
    p_scan_pub.publish(rscan);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "msg_parser");

    ros::NodeHandle n;
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    ros::Publisher scan_pub = n.advertise<sensor_msgs::LaserScan>("scan", 50);
    ros::Subscriber odom_read = n.subscribe("odom_partial", 1000, futzOdom);
    ros::Subscriber scan_read = n.subscribe("ir_raw", 1000, irToLaser);
    
    tf::TransformBroadcaster odom_broadcaster;
    tf::TransformBroadcaster scan_broadcaster;

    p_odom_pub = &odom_pub;
    p_scan_pub = &scan_pub;
    p_odom_broadcaster = &odom_broadcaster;
    p_scan_broadcaster = &scan_broadcaster;

    ros::spin();
}
