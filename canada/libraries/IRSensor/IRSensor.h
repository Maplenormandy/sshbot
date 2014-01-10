
#ifndef IRSENSOR_H_
#define IRSENSOR_H_

#include <inttypes.h>

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

#endif
