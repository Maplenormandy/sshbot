#include <ros.h>
#include <Encoder.h>
#include <PinSettings.h>
#include <SensorTimer.h>
#include <DifferentialDrive.h>
#include <IRSensor.h>
#include <Servo.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int16.h>

std_msgs::String str_msg;

ros::NodeHandle nh;
ros::Publisher chatter("chatter", &str_msg);

DifferentialDrive dd;
IRSuite irs;

void odom_reset_cb(const std_msgs::Empty& msg)
{
    dd.reset();
}
ros::Subscriber<std_msgs::Empty> odom_reset("odom_reset", &odom_reset_cb);

void sas_cmd_cb(const std_msgs::Float32& msg)
{
    uint16 pwm = (uint16) constrain(abs(msg.data*65535), 0, 65535);
    uint8 dir = msg.data > 0.0f ? HIGH : LOW;

    digitalWrite(SAS_DIR, dir);
    pwmWrite(SAS_PWM, pwm);
}
ros::Subscriber<std_msgs::Float32> sas_cmd("sas_cmd", &sas_cmd_cb);

void roller_cmd_cb(const std_msgs::Float32& msg)
{
    uint16 pwm = (uint16) constrain(abs(msg.data*65535), 0, 65535);
    uint8 dir = msg.data > 0.0f ? HIGH : LOW;

    digitalWrite(ROLLER_DIR, dir);
    pwmWrite(ROLLER_PWM, pwm);
}
ros::Subscriber<std_msgs::Float32> roller_cmd("roller_cmd", &roller_cmd_cb);

void screw_cmd_cb(const std_msgs::Float32& msg)
{
    uint16 pwm = (uint16) constrain(abs(msg.data*65535), 0, 65535);
    uint8 dir = msg.data > 0.0f ? HIGH : LOW;

    digitalWrite(SCREW_DIR, dir);
    pwmWrite(SCREW_PWM, pwm);
}
ros::Subscriber<std_msgs::Float32> screw_cmd("screw_cmd", &screw_cmd_cb);

Servo kick;
void kick_cmd_cb(const std_msgs::Int16& msg)
{
    kick.write(msg.data);
}
ros::Subscriber<std_msgs::Int16> kick_cmd("kick_cmd", &kick_cmd_cb);

Servo pac;
void pac_cmd_cb(const std_msgs::Int16& msg)
{
    pac.write(msg.data);
}
ros::Subscriber<std_msgs::Int16> pac_cmd("pac_cmd", &pac_cmd_cb);

Servo gate_g;
void gate_g_cmd_cb(const std_msgs::Int16& msg)
{
    gate_g.write(msg.data);
}
ros::Subscriber<std_msgs::Int16> gate_g_cmd("gate_g_cmd", &gate_g_cmd_cb);

Servo gate_r;
void gate_r_cmd_cb(const std_msgs::Int16& msg)
{
    gate_r.write(msg.data);
}
ros::Subscriber<std_msgs::Int16> gate_r_cmd("gate_r_cmd", &gate_r_cmd_cb);

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
    nh.advertise(dd.overspeed);

    nh.subscribe(odom_reset);
    nh.subscribe(sas_cmd);
    nh.subscribe(roller_cmd);
    nh.subscribe(screw_cmd);
    nh.subscribe(kick_cmd);
    nh.subscribe(pac_cmd);
    nh.subscribe(gate_g_cmd);
    nh.subscribe(gate_r_cmd);

    kick.attach(KICK_SERVO);
    kick.write(0);
    pac.attach(PAC_SERVO);
    pac.write(70);
    gate_g.attach(GATE_G_SERVO);
    gate_g.write(15);
    gate_r.attach(GATE_R_SERVO);
    gate_r.write(90);

    irs.l.fwd.calibrate(0.00379680814028f, -0.37963064697f,
            0.1f, 0.8f);
    irs.l.fwd.attach(IR_L_FWD);

    irs.l.mid.calibrate(0.00407376233424f, -0.935821200893f,
            0.1f, 0.8f);
    irs.l.mid.attach(IR_L_MID);

    irs.l.bak.calibrate(0.00363762465628f, -0.0852030557814f,
            0.1f, 0.8f);
    irs.l.bak.attach(IR_L_BAK);

    irs.r.fwd.calibrate(0.00369894253316f, -0.975761919236f,
            0.1f, 0.8f);
    irs.r.fwd.attach(IR_R_FWD);
    irs.r.mid.calibrate(0.00382965270558f, -0.988533304527f,
            0.1f, 0.8f);
    irs.r.mid.attach(IR_R_MID);
    irs.r.bak.calibrate(0.00382557011975f, -0.908450098318f,
            0.1f, 0.8f);
    irs.r.bak.attach(IR_R_BAK);

    irs.fwd_l.attach(IR_FWD_L);
    irs.fwd_l.calibrate(0.00640927906921f, 0.00452178178792f,
            0.04f, 0.3f);
    irs.fwd_r.attach(IR_FWD_R);
    irs.fwd_r.calibrate(0.00672864575128f, -0.0114986048003f,
            0.04f, 0.3f);


    digitalWrite(ROLLER_DIR, HIGH);
    pwmWrite(ROLLER_PWM, 0);

    digitalWrite(SCREW_DIR, HIGH);
    pwmWrite(SCREW_PWM, 0);

    digitalWrite(SAS_DIR, HIGH);
    pwmWrite(SAS_PWM, 0);
    
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

