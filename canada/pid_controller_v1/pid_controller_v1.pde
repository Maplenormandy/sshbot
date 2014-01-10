#include <ros.h>
#include <Encoder.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/String.h>

const uint8 MOTOR_L_PWM = 0; // PWM
const uint8 MOTOR_L_DIR = 1; // OUTPUT

const uint8 MOTOR_R_PWM = 2; // PWM
const uint8 MOTOR_R_DIR = 3; // OUTPUT

const uint8 ENC_L_A = 32; // INPUT
const uint8 ENC_L_B = 31; // INPUT
const uint8 ENC_R_A = 34; // INPUT
const uint8 ENC_R_B = 33; // INPUT
const uint8 ENC_GND = 36; // OUTPUT, LOW
const uint8 ENC_VCC = 37; // OUTPUT, HIGH

geometry_msgs::TwistStamped odom_msg;
geometry_msgs::Twist cmd_vel;

std_msgs::String str_msg;

void cmdCb(const geometry_msgs::Twist& msg)
{
    cmd_vel = msg;
}


// Gains
double kp = 150000, kd = 12000;
double kisat = PI;
double lowpass = 0.7;

void pidCb(const geometry_msgs::Twist& msg)
{
    kp = msg.linear.x;
    kd = msg.linear.y;
    kisat = msg.linear.z;
    lowpass = msg.angular.x;
}

ros::NodeHandle nh;
ros::Publisher odom("odom_partial", &odom_msg);
ros::Publisher chatter("chatter", &str_msg);
ros::Subscriber<geometry_msgs::Twist> velocity_cmds("cmd_vel", &cmdCb );
ros::Subscriber<geometry_msgs::Twist> pid_tune("pid_tune", &pidCb );

Encoder encL, encR;

void setup()
{
    pinMode(BOARD_LED_PIN, OUTPUT);

    pinMode(MOTOR_L_PWM, PWM);
    pinMode(MOTOR_L_DIR, OUTPUT);
    pinMode(MOTOR_R_PWM, PWM);
    pinMode(MOTOR_R_DIR, OUTPUT);

    pinMode(ENC_L_A, INPUT);
    pinMode(ENC_L_B, INPUT);
    pinMode(ENC_R_A, INPUT);
    pinMode(ENC_R_B, INPUT);

    pinMode(ENC_GND, OUTPUT);
    pinMode(ENC_VCC, OUTPUT);
    
    digitalWrite(ENC_GND, LOW);
    digitalWrite(ENC_VCC, HIGH);

    encL.attach(ENC_L_A, ENC_L_B);
    encR.attach(ENC_R_A, ENC_R_B);
    encL.write(0);
    encR.write(0);

    nh.initNode();
    nh.subscribe(velocity_cmds);
    nh.subscribe(pid_tune);
    nh.advertise(chatter);
    nh.advertise(odom);

    odom_msg.header.frame_id = "READ_THE_DOCS";

    toggleLED();
}
/*
 * y - measured value
 */
// Distance values
double yl = 0;
double yr = 0;
// Integrated distance values
double yil = 0;
double yir = 0;
// Last distance values
double yll = 0;
double ylr = 0;
// Last filtered derivative value
double ydll = 0;
double ydlr = 0;

/*
 * r - signal value
 */
// Signal value
double rdl = 0;
double rdr = 0;
// Integrated signal value
double rl = 0;
double rr = 0;
// Last signal value
double rll = 0;
double rlr = 0;

/*
 * Pose
 */
double th = 0;
double x = 0;
double y = 0;
uint32 seq = 0;

uint32 lasttime = 0;
uint32 time = 0;


const double wheelR = 0.0492125; // 3 7/8" diam in meters
const double axleL = .105; // halfway across bot in meters
const double convFactor = 2.0*PI/64.0/29.0;

const uint32 delaytime = 16667;
//const uint32 delaytime = 30000;

void loop()
{

    // Encoder values
    long el = encL.read();
    long er = encR.read();

    // Precise timekeeping
    lasttime = time;
    time = micros();
    odom_msg.header.stamp.sec = time/1000000 + nh.sec_offset;
    odom_msg.header.stamp.nsec = (time%1000000)*1000UL + nh.nsec_offset;
    ros::normalizeSecNSec(odom_msg.header.stamp.sec, odom_msg.header.stamp.nsec);
    odom_msg.header.seq = seq;
    ++seq;
    double dt = (time-lasttime)/1000000.0;

    // Convert to angular values
    yl = el*convFactor;
    // Right encoder should be flipped
    yr = -er*convFactor;

    // Calculate derivatives
    double ydl = (yl - yll) / dt;
    double ydr = (yr - ylr) / dt;

    ydl = lowpass*ydll + (1-lowpass)*ydl;
    ydr = lowpass*ydlr + (1-lowpass)*ydr;

    yll = yl;
    ylr = yr;
    ydll = ydl;
    ydlr = ydr;

    // Report the calculated velocity
    // TODO Higher order calculation
    double esym = (ydl+ydr) * wheelR / 2;
    double eanti = (ydr-ydl) * wheelR / axleR / 2;
    odom_msg.twist.linear.x = esym;
    odom_msg.twist.angular.z = eanti;

    // Calculate the updated pose
    // TODO Higher order calculation
    x += esym * cos(th) * dt;
    y += esym * sin(th) * dt;
    th += eanti * dt;

    // Report the calculated pose
    odom_msg.twist.linear.z = th;
    odom_msg.twist.angular.x = x;
    odom_msg.twist.angular.y = y;

    // Turn commanded velocity to L/R commands
    double sym = cmd_vel.linear.x/wheelR;
    double anti = cmd_vel.angular.z*axleR/wheelR;
    rdl = sym-anti;
    rdr = sym+anti;

    // Integrate velocity command
    rl += rdl*dt;
    rr += rdr*dt;

    // Saturation
    if (abs(rl-yl)>kisat || abs(rr-yr)>kisat)
    {
        str_msg.data = "Windup";
        chatter.publish(&str_msg);
    }
    rl = constrain(rl, yl-kisat,yl+kisat);
    rr = constrain(rr, yr-kisat,yr+kisat);

    // command
    double outl = (rl-yl)*kp + (rdl-ydl)*kd;
    double outr = (rr-yr)*kp + (rdr-ydr)*kd;

    // Output to motors
    // HIGH drives forward
    digitalWrite(MOTOR_L_DIR, outl < 0 ? LOW : HIGH);
    digitalWrite(MOTOR_R_DIR, outr < 0 ? LOW : HIGH);
    pwmWrite(MOTOR_L_PWM, (uint16) constrain(abs(outl), 0, 64000));
    pwmWrite(MOTOR_R_PWM, (uint16) constrain(abs(outr), 0, 64000));

    odom.publish(&odom_msg);

    toggleLED();

    nh.spinOnce();

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
