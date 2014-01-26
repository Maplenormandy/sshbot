#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <cmath>
#include <boost/math/constants/constants.hpp>

const double pi = boost::math::constants::pi<double>();

int main(int argc, char** argv){
  ros::init(argc, argv, "tf_broadcaster");
  ros::NodeHandle n;

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

  ros::Rate r(20.0);

  tf::TransformBroadcaster broadcaster;

  while(n.ok()){
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::createQuaternionFromYaw(-pi/2.0),
            tf::Vector3(0.02032, -0.0851408, 0.1016)),
        ros::Time::now(),"base_link", "rscan"));
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::createQuaternionFromYaw(pi/2.0),
            tf::Vector3(0.02032, 0.0851408, 0.1016)),
        ros::Time::now(),"base_link", "lscan"));

    // TODO Fix these values
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.2, -0.1, 0.1016)),
        ros::Time::now(),"base_link", "flscan"));
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.2, 0.1, 0.1016)),
        ros::Time::now(),"base_link", "frscan"));

    // TODO Fix these values
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.2, 0.0, 0.0)),
        ros::Time::now(),"base_link", "camera_link"));
    r.sleep();
  }
}
