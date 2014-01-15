#ifndef IRARRAY_H_
#define IRARRAY_H_

#include <SensorTimer.h>
#include <IRSensor.h>

#include <ros.h>
#include <sensor_msgs/LaserScan.h>

class IRArray
{
    public:
        const static int NUM_IRS = 3;
        const static float lowpass = 1.0;

        IRArray();

        IRSensor sensors[NUM_IRS];
        float rangesl[NUM_IRS];
        float ranges[NUM_IRS];

        sensor_msgs::LaserScan ir_msg;
        ros::Publisher ir_data;

        SensorTimer st;

        /*
         * Functions
         */
        void loop(void);
};

#endif

