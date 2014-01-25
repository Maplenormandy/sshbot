#ifndef SENSOR_TIMER_H_
#define SENSOR_TIMER_H_

#include <ros.h>
#include <std_msgs/Header.h>

struct SensorTimer
{
    uint32 now;
    uint32 old;

    double dt;

    SensorTimer(std_msgs::Header& _header);

    std_msgs::Header& header;

    static ros::NodeHandle* nh;

    static void init(ros::NodeHandle* _nh);

    void update(uint32 time);
};

#endif
