#include "IRArray.h"

#include <wirish.h>
#include <PinSettings.h>
#include <math.h>

char frameid[5] = "odom";

IRArray::IRArray() :
    ir_data("ir_data", &ir_msg),
    st(ir_msg.header)
{
    for (int i = 0; i < NUM_IRS; ++i)
    {
        sensors[i].attach(IRSENSORS[i]);
        ranges[i] = HUGE_VALF;
        rangesl[i] = 0.3f;
    }

    ir_msg.header.frame_id = frameid;
    // TODO Figure out some way to handle two laser scans
    ir_msg.angle_min = (float) (0.25*PI);
    ir_msg.angle_max = (float) (0.75*PI);
    ir_msg.angle_increment = (float) (0.25*PI);
    ir_msg.scan_time = 0.0f;
    ir_msg.range_min = 0.10f;
    ir_msg.range_max = 0.80f;
    ir_msg.ranges = ranges;
    ir_msg.ranges_length = 3;
}

void IRArray::loop(void)
{
    for (int i = 0; i < NUM_IRS; ++i)
    {
        float val = sensors[i].read();
        if (isfinite(val))
        {
            ranges[i] = val * lowpass + rangesl[i] * (1-lowpass);
            rangesl[i] = ranges[i];
        }
    }
    st.update(micros());
    ir_msg.scan_time = (float) st.dt;
    ir_data.publish(&ir_msg);
}
