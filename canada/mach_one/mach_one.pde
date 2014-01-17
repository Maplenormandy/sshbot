#include <ros.h>
#include <Encoder.h>
#include <PinSettings.h>
#include <SensorTimer.h>
#include <DifferentialDrive.h>
#include <IRSensor.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>

std_msgs::String str_msg;
std_msgs::Float32 front_msg;

ros::NodeHandle nh;
ros::Publisher chatter("chatter", &str_msg);

DifferentialDrive dd;
IRSuite irs;

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

    nh.advertise(irs.ir_raw);

    nh.subscribe(dd.cmd_vel);
    nh.subscribe(dd.pid_tune);
    nh.advertise(dd.odom);

    irs.l.fwd.calibrate(0.00379680814028f, -0.37963064697f,
            0.1f, 0.8f);
    irs.l.fwd.attach(IR_L_FWD);

    irs.l.mid.calibrate(0.00407376233424f, -0.935821200893f,
            0.1f, 0.8f);
    irs.l.mid.attach(IR_L_MID);

    irs.l.bak.calibrate(0.00363762465628f, -0.0852030557814f,
            0.1f, 0.8f);
    irs.l.bak.attach(IR_L_BAK);

    irs.fwd.attach(IR_FWD);
    irs.fwd.calibrate(0.00550560406861f, 0.889300680702f,
            0.1f, 0.8f);

    // Dagu motor controller, for front roller
    pinMode(6, PWM);
    pinMode(7, OUTPUT);
    digitalWrite(7, HIGH);
    analogWrite(6, 40000);
    
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
    if (seq%3 == 0)
    {
        irs.loop();
    }

    nh.spinOnce();

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
