#include "ros/ros.h"

#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

#include <cmath>

/*
 * Uses IR data from the wallfollower to follow the wall.
 */

geometry_msgs::Twist vel;

ros::Publisher cmd_vel;

ros::NodeHandle *nh;

void irCb(const sensor_msgs::LaserScan& msg)
{
    float a = msg.ranges[0] - 0.3f;
    float b = msg.ranges[1] - 0.3f;

    if (std::isinf(a)) {
        a = a<0.0f ? 0.1f : 0.8f;
    }
    if (std::isinf(b)) {
        b = b<0.0f ? 0.1f : 0.8f;
    }

    double kp = 2.0;
    double bias = 1.5;
    nh->getParamCached("wallfollower/kp", kp);
    nh->getParamCached("wallfollower/bias", bias);

    vel.angular.z = kp * (a*bias - b);
    cmd_vel.publish(vel);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wallfollower");
    ros::NodeHandle n;
    nh = &n;

    cmd_vel = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    ros::Subscriber ir_data = n.subscribe("/ir_data", 1000, &irCb);

    vel.linear.x = 0.2;

    ROS_INFO("Ready!");

    ros::spin();
}

