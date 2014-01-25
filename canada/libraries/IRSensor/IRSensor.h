
#ifndef IRSENSOR_H_
#define IRSENSOR_H_

#include <stdlib.h>
#include <inttypes.h>
#include <b2b/IRStamped.h>
#include <SensorTimer.h>

class IRSensor
{
    protected:
        float slope;
        float intercept;

        float minRange;
        float maxRange;

        uint8_t pin;
    public:
        IRSensor();

        void calibrate(float _slope, float _intercept,
                float minRange, float maxRange);

        // Attaches the IR Sensor to a pin. Returns 0 if failure,
        // which should never happen.
        uint8_t attach(uint8_t pinIn);
        // Reads a distance. Returns -Inf if too close or +Inf if too far
        float  read(void);
};

class IRSuite
{
    public:
        IRSuite();

        b2b::IRStamped ir_msg;
        ros::Publisher ir_raw;

        SensorTimer st;

        struct IRArray
        {
            IRSensor fwd, mid, bak;
        } l, r;

        IRSensor fwd_l;
        IRSensor fwd_r;

        /*
         * Functions
         */
        void loop(void);
};

#endif
