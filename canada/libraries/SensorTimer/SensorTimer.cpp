#include "SensorTimer.h"

ros::NodeHandle* SensorTimer::nh;

SensorTimer::SensorTimer(std_msgs::Header& _header) :
    header(_header)
{
    old = 0;
    now = 0;
    header.seq = 0;
}

void SensorTimer::init(ros::NodeHandle* _nh)
{
    nh = _nh;
}

void SensorTimer::update(uint32_t time)
{
    old = now;
    now = time;
    dt = (now-old)/1000000.0;
    header.stamp.sec = now/1000000 + nh->sec_offset;
    header.stamp.nsec = (now%1000000)*1000UL + nh->nsec_offset;
    ros::normalizeSecNSec(header.stamp.sec, header.stamp.nsec);
    header.seq += 1;
}
