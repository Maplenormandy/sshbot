#include <usb_serial.h>
#include <wirish_time.h>

#ifndef ROS_MAPLE_HARDWARE_H_
#define ROS_MAPLE_HARDWARE_H_

class MapleHardware
{
    public:
        MapleHardware()
        {
        }

        // any initialization code necessary to use the serial port
        void init()
        {
            SerialUSB.begin();
        }

        // read a byte from the serial port. -1 = failure
        int read()
        {
            if (SerialUSB.available() > 0)
            {
                return SerialUSB.read();
            }
            else
            {
                return -1;
            }
        }

        // write data to the connection to ROS
        void write(uint8_t* data, int length)
        {
            for (int i = 0; i < length; ++i)
                SerialUSB.write(data[i]);
        }

        // returns milliseconds since start of program
        unsigned long time()
        {
            return millis();
        }

};

#endif
