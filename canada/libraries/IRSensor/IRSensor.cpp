
#include "IRSensor.h"

#include <wirish.h>

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
        return -HUGE_VALF;
    }
    else if (dist > maxRange)
    {
        return HUGE_VALF;
    }
    return dist;
}

IRSuite::IRSuite() :
    ir_raw("ir_raw", &ir_msg),
    st(ir_msg.header)
{
}

void IRSuite::loop(void)
{
    ir_msg.l.fwd = l.fwd.read();
    ir_msg.l.mid = l.mid.read();
    ir_msg.l.bak = l.bak.read();
    ir_msg.r.fwd = r.fwd.read();
    ir_msg.r.mid = r.mid.read();
    ir_msg.r.bak = r.bak.read();
    //ir_msg.fwd_l = fwd_l.read();
    //ir_msg.fwd_r = fwd_r.read();

    st.update(micros());
    ir_raw.publish(&ir_msg);
}
