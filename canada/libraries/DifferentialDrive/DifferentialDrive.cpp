#include <PinSettings.h>

#include "DifferentialDrive.h"

double DifferentialDrive::kp;
double DifferentialDrive::kd;
double DifferentialDrive::lowpass;
double DifferentialDrive::int_sat;

geometry_msgs::Twist DifferentialDrive::cmd_vel_msg;

void DifferentialDrive::pidTuneCb(const geometry_msgs::Twist& msg)
{
    kp = msg.linear.x;
    kd = msg.linear.y;
    int_sat = msg.linear.z;
    lowpass = msg.angular.x;
}

void DifferentialDrive::cmdCb(const geometry_msgs::Twist& msg)
{
    cmd_vel_msg = msg;
}

char read_the_docs[14] = "READ_THE_DOCS";

DifferentialDrive::DifferentialDrive() :
    odom_msg(),
    odom("odom_partial", &odom_msg),
    pid_tune("pid_tune", &pidTuneCb),
    cmd_vel("cmd_vel", &cmdCb),
    overspeed("overspeed", &overspeed_msg),
    st(odom_msg.header)
{
    encL.attach(ENC_L_A, ENC_L_B);
    encR.attach(ENC_R_A, ENC_R_B);

    //odom_msg.header.frame_id = read_the_docs;
}

void DifferentialDrive::reset(void)
{
    kp = 180000;
    kd = 11000;
    lowpass = 0.7;
    int_sat = 70000.0/kp;

    yl = 0;
    yr = 0;
    yil = 0;
    yir = 0;
    yll = 0;
    ylr = 0;
    ydll = 0;
    ydlr = 0;
    rdl = 0;
    rdr = 0;
    rl = 0;
    rr = 0;
    rll = 0;
    rlr = 0;
    th = 0;
    x = 0;
    y = 0;

    cmd_vel_msg.linear.x = 0.0;
    cmd_vel_msg.angular.z = 0.0;

    encL.write(0);
    encR.write(0);
}

void DifferentialDrive::loop(void)
{
    // Encoder values
    long el = encL.read();
    long er = encR.read();
    st.update(micros());

    // Convert to angular values
    yl = el*convFactor;
    // Right encoder should be flipped
    yr = -er*convFactor;

    // Calculate derivatives
    double ydl = (yl - yll) / st.dt;
    double ydr = (yr - ylr) / st.dt;

    ydl = (1-lowpass)*ydll + lowpass*ydl;
    ydr = (1-lowpass)*ydlr + lowpass*ydr;

    yll = yl;
    ylr = yr;
    ydll = ydl;
    ydlr = ydr;

    // Report the calculated velocity
    // TODO Higher order calculation
    double esym = (ydl+ydr) * wheelR;
    double eanti = (ydr-ydl) * wheelR / axleR;
    odom_msg.twist.linear.x = esym;
    odom_msg.twist.angular.z = eanti;

    // Calculate the updated pose
    // TODO Higher order calculation
    // Currently: x,y 2nd order, th 1st order
    x += esym * cos(th) * st.dt / 2.0;
    y += esym * sin(th) * st.dt / 2.0;
    th += eanti * st.dt;
    x += esym * cos(th) * st.dt / 2.0;
    y += esym * sin(th) * st.dt / 2.0;

    // Report the calculated pose
    odom_msg.twist.linear.z = th;
    odom_msg.twist.angular.x = x;
    odom_msg.twist.angular.y = y;

    // Turn commanded velocity to L/R commands
    double sym = cmd_vel_msg.linear.x/wheelR;
    double anti = cmd_vel_msg.angular.z*axleR/wheelR;
    rdl = (sym-anti)/2.0;
    rdr = (sym+anti)/2.0;

    // Integrate velocity command
    rl += rdl*st.dt;
    rr += rdr*st.dt;

    // Saturation
    if (abs(rl-yl)>int_sat || abs(rr-yr)>int_sat)
    {
        overspeed.publish(&overspeed_msg);
    }
    rl = constrain(rl, yl-int_sat,yl+int_sat);
    rr = constrain(rr, yr-int_sat,yr+int_sat);

    // command
    double outl = (rl-yl)*kp + (rdl-ydl)*kd;
    double outr = (rr-yr)*kp + (rdr-ydr)*kd;

    // Output to motors
    // LOW drives forward
    digitalWrite(MOTOR_L_DIR, outl > 0 ? HIGH : LOW);
    digitalWrite(MOTOR_R_DIR, outr > 0 ? HIGH : LOW);
    pwmWrite(MOTOR_L_PWM, (uint16) constrain(abs(outl), 0, 65535));
    pwmWrite(MOTOR_R_PWM, (uint16) constrain(abs(outr), 0, 65535));
    //pwmWrite(MOTOR_L_PWM, (uint16) abs(cmd_vel_msg.linear.x));
    //pwmWrite(MOTOR_R_PWM, (uint16) abs(cmd_vel_msg.linear.y));
    //pwmWrite(MOTOR_L_PWM, (uint16) (micros()%65536));
    //pwmWrite(MOTOR_R_PWM, (uint16) (micros()%65536));

    odom.publish(&odom_msg);
}

