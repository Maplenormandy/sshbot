#include <ros.h>
#include <Encoder.h>
#include <PinSettings.h>
#include <SensorTimer.h>
#include <DifferentialDrive.h>
#include <IRSensor.h>
#include <IRArray.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/String.h>

std_msgs::String str_msg;

ros::NodeHandle nh;
ros::Publisher chatter("chatter", &str_msg);

DifferentialDrive dd;
IRArray irs;

void deb()
{
    delay(100);
    toggleLED();
}

void setup()
{
    pinSetup();

    deb();
    nh.initNode();
    nh.advertise(chatter);

    deb();
    SensorTimer::init(&nh);

    deb();
    nh.advertise(irs.ir_data);

    nh.subscribe(dd.cmd_vel);
    nh.subscribe(dd.pid_tune);
    nh.advertise(dd.odom);

    irs.sensors[0].calibrate(0.002372585744f, 1.37669143987f,
            0.1f, 0.8f);
    irs.sensors[1].calibrate(0.00264713840204f, 1.04544891075f,
            0.1f, 0.8f);
    
    deb();
    dd.reset();

    toggleLED();
}

uint32 seq = 0;
uint32 time = 0;

const uint32 delaytime = 16667;
//const uint32 delaytime = 30000;

void loop()
{
    // Precise timekeeping
    time = micros();

    dd.loop();
    irs.loop();

    nh.spinOnce();

    ++seq;
    toggleLED();
    uint32 donetime = micros();

    if (donetime < time+delaytime)
    {
        delayMicroseconds(time+delaytime - donetime);
    }
    else
    {
        //delay(100);
        str_msg.data = "Frame";
        chatter.publish(&str_msg);
    }
}
