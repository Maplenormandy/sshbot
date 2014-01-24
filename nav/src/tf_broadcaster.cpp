#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <cmath>
#include <boost/math/constants/constants.hpp>

const double pi = boost::math::constants::pi<double>();

int main(int argc, char** argv){
  ros::init(argc, argv, "base_link_to_map_tf");
  ros::NodeHandle n;

  ros::Rate r(1.0);

  tf::TransformBroadcaster broadcaster;

  while(n.ok()){
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::createQuaternionFromYaw(pi/2.0),
            tf::Vector3(0.02032, -0.0851408, 0.0)),
        ros::Time::now(),"base_link", "rscan"));
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::createQuaternionFromYaw(-pi/2.0),
            tf::Vector3(0.02032, 0.0851408, 0.0)),
        ros::Time::now(),"base_link", "lscan"));

    // TODO Fix these values
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.2, -0.1, 0.0)),
        ros::Time::now(),"base_link", "flscan"));
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.2, 0.1, 0.0)),
        ros::Time::now(),"base_link", "frscan"));

    // TODO Fix these values
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.2, 0.0, 0.0)),
        ros::Time::now(),"base_link", "camera_link"));
    r.sleep();
  }
}
