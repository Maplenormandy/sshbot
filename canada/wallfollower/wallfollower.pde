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
#include <std_msgs/Float32.h>

std_msgs::String str_msg;
std_msgs::Float32 front_msg;

ros::NodeHandle nh;
ros::Publisher chatter("chatter", &str_msg);
ros::Publisher front_ir("front_ir", &front_msg);

DifferentialDrive dd;
IRArray irs;
IRSensor front;

void deb()
{
    delay(100);
    toggleLED();
}

void setup()
{
    pinSetup();

    nh.initNode();
    nh.advertise(chatter);

    SensorTimer::init(&nh);

    nh.advertise(irs.ir_data);

    nh.subscribe(dd.cmd_vel);
    nh.subscribe(dd.pid_tune);
    nh.advertise(dd.odom);
    
    nh.advertise(front_ir);

    irs.sensors[0].calibrate(0.00363762465628f, -0.0852030557814f,
            0.1f, 0.8f);
    irs.sensors[1].calibrate(0.00407376233424f, -0.935821200893f,
            0.1f, 0.8f);
    irs.sensors[2].calibrate(0.00379680814028f, -0.37963064697f,
            0.1f, 0.8f);

    front.attach(18);

    front.calibrate(0.00550560406861f, 0.889300680702f,
            0.1f, 0.8f);

    
    dd.reset();

    toggleLED();
}

uint32 seq = 0;
uint32 time = 0;

const uint32 delaytime = 15000;
//const uint32 delaytime = 30000;

void loop()
{
    // Precise timekeeping
    time = micros();

    dd.loop();
    irs.loop();

    nh.spinOnce();

    front_msg.data = front.read();

    front_ir.publish(&front_msg);

    if (digitalRead(BOARD_BUTTON_PIN) == HIGH)
    {
        dd.reset();
    }

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
