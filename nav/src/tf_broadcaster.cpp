#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <cmath>
#include <std_msgs/Empty.h>
#include <std_msgs/Int16.h>
#include <boost/math/constants/constants.hpp>

const double pi = boost::math::constants::pi<double>();

int main(int argc, char** argv){
    ros::init(argc, argv, "tf_broadcaster");
    ros::NodeHandle n;

    /*
    ros::Publisher initpos = 
        n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 15, true);
    geometry_msgs::PoseWithCovarianceStamped pose;
    tf::Quaternion q = tf::createQuaternionFromYaw(pi);
    geometry_msgs::Quaternion qMsg;
    tf::quaternionTFToMsg(q, qMsg);
    pose.header.frame_id = "map";
    pose.header.stamp = ros::Time::now();
    pose.pose.pose.orientation = qMsg;
    pose.pose.pose.position.x = 0.558798517172;
    pose.pose.pose.position.y = 0.558798517172;
    initpos.publish(pose);
    */

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat =
        tf::createQuaternionMsgFromYaw(0.0);
    //geometry_msgs::Quaternion odom_quat =
    //    tf::createQuaternionMsgFromYaw(0.0);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.frame_id = "odom_fake";
    odom_trans.child_frame_id = "odom";

    // Get transformation from odom coordinates. Note that the odom_partial
    // message is strange.
    odom_trans.transform.translation.x = 0.558798517172;
    odom_trans.transform.translation.y = 0.558798517172;
    //odom_trans.transform.translation.x = 0.0;
    //odom_trans.transform.translation.y = 0.0;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;


    tf::TransformBroadcaster odom_broadcaster;

    std_msgs::Int16 resetMsg;
    ros::Publisher resetPub =
        n.advertise<std_msgs::Int16>("/odom_reset", 15, true);
    resetPub.publish(resetMsg);

    ros::Rate r(20.0);

    tf::TransformBroadcaster broadcaster;

    while(n.ok()){
        broadcaster.sendTransform(
                tf::StampedTransform(
                    tf::Transform(tf::createQuaternionFromYaw(-pi/2.0),
                        tf::Vector3(0.02032, -0.0851408, 0.0)),
                    ros::Time::now(),"base_link", "rscan"));
        broadcaster.sendTransform(
                tf::StampedTransform(
                    tf::Transform(tf::createQuaternionFromYaw(pi/2.0),
                        tf::Vector3(0.02032, 0.0851408, 0.0)),
                    ros::Time::now(),"base_link", "lscan"));

        //send the transform
        odom_trans.header.stamp = ros::Time::now();
        //broadcaster.sendTransform(odom_trans);

        // TODO Fix these values
        broadcaster.sendTransform(
                tf::StampedTransform(
                    tf::Transform(tf::Quaternion(0, 0, 0, 1),
                        tf::Vector3(0.14605, 0.117475, 0.0)),
                    ros::Time::now(),"base_link", "flscan"));
        broadcaster.sendTransform(
                tf::StampedTransform(
                    tf::Transform(tf::Quaternion(0, 0, 0, 1),
                        tf::Vector3(0.14605, -0.117475, 0.0)),
                    ros::Time::now(),"base_link", "frscan"));

        broadcaster.sendTransform(
                tf::StampedTransform(
                    tf::Transform(tf::Quaternion(0, 0, 0, 1),
                        tf::Vector3(0.14605-0.2, 0.0, 0.0)),
                    ros::Time::now(),"base_link", "fscan"));

        // TODO Fix these values
        broadcaster.sendTransform(
                tf::StampedTransform(
                    tf::Transform(tf::Quaternion(0, 0, 0, 1),
                        tf::Vector3(0.14605, 0.0, 0.0)),
                    ros::Time::now(),"base_link", "camera_link"));
        r.sleep();
    }
}
