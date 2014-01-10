#include <wirish.h>

#include "IRSensor.h"

IRSensor::IRSensor()
{
}

void IRSensor::calibrate(float _slope, float _intercept,
        float _minRange, float _maxRange)
{
    slope = _slope;
    intercept = _intercept;
    minRange = _minRange;
    maxRange = _maxRange;
}

uint8_t IRSensor::attach(uint8_t pinIn)
{
    pin = pinIn;
    pinMode(pin, INPUT);
    return 1;
}

float IRSensor::read(void)
{
    uint16_t val = analogRead(pin);
    float dist = 1.0 / (slope*val+intercept);
    if (dist < minRange)
    {
        return -HUGE_VAL;
    }
    else if (dist > maxRange)
    {
        return HUGE_VAL;
    }
    return dist;
}

