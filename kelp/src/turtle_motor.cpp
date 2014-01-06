#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/Twist.h"

#include <algorithm>
#include <cmath>

/*
 * Simulates a single motor on turtlebot.
 *
 * Model taken from http://ctms.engin.umich.edu/CTMS/index.php?example=MotorSpeed&section=SystemModeling
 * Parameter names can be taken from there
 */
// Motor Params
double J = 0.01;
double b = 0.1;
double Ke = 0.01;
double Kt = 0.01;
double R = 1;
double L = 0.5;

double V_MAX = 5.0;

struct Motor
{

    // State variables
    double i;
    double thdot;
    // For the "encoder"
    double th;

    // Input variable, ZOH (i.e. it stays where you set it)
    double V;

    Motor()
    {
        i = 0.0;
        thdot = 0.0;
        th = 0.0;
    }

    // updates the motor's state given a timestep dt
    void update(double dt)
    {
        // Eww. Forward Euler. How's that for signal noise?
        double dthdotdt = -b/J * thdot + Kt/J * i;
        double didt = -Ke/L * thdot + -R/L * i + 1.0/L * V;
        th += thdot * dt;
        thdot += dthdotdt * dt;
        i += didt * dt;
    }

    void updateVoltage(const std_msgs::Float32& msg) {
        V = std::min(V_MAX, std::max(-V_MAX, (double) msg.data));
    }
} motorL, motorR;

int main(int argc, char **argv)
{
    // ROS init code. See tutorials
    ros::init(argc, argv, "turtle_motor");
    ros::NodeHandle n;

    // Publish to two channels, both floats, which is the encoder data
    ros::Publisher encL = n.advertise<std_msgs::Float32>("enc_left", 1000);
    ros::Publisher encR = n.advertise<std_msgs::Float32>("enc_right", 1000);
    ros::Publisher turtleVel = n.advertise<geometry_msgs::Twist>
        ("turtle1/cmd_vel", 1000);

    // Subscribe to the voltage update channels 
    ros::Subscriber subL = n.subscribe("voltage_left", 1000,
            &Motor::updateVoltage, &motorL);
    ros::Subscriber subR = n.subscribe("voltage_right", 1000,
            &Motor::updateVoltage, &motorR);

    ros::Rate loop_rate(60);

    // Angle the robot is pointing in
    double robotTh = 0.0;

    double wheelRad = 4.0;
    double axleLen = 0.25;

    ROS_INFO("Ready!");

    while (ros::ok())
    {
        // Update parameters
        n.getParamCached("robot/wheel_radius", wheelRad);
        n.getParamCached("robot/axle_length", axleLen);
        n.getParamCached("motor/v_max", V_MAX);
        n.getParamCached("motor/inductance", L);
        n.getParamCached("motor/resistance", R);
        n.getParamCached("motor/torque_constant", Kt);
        n.getParamCached("motor/emf_constant", Ke);
        n.getParamCached("motor/viscous_friction", b);
        n.getParamCached("motor/rotor_inertia", J);

        std_msgs::Float32 encLmsg, encRmsg;
        encLmsg.data = (float) motorL.th;
        encRmsg.data = (float) motorR.th;

        encL.publish(encLmsg);
        encR.publish(encRmsg);

        // Update the turtle's velocity based on the motor velocities
        double forwardVel = (motorL.thdot + motorR.thdot) / 2 * wheelRad;
        double turnRate = (motorR.thdot - motorL.thdot) / 2 * wheelRad / axleLen;

        geometry_msgs::Twist velMsg;
        velMsg.linear.x = forwardVel;
        velMsg.linear.y = 0;
        velMsg.linear.z = 0;
        velMsg.angular.x = 0;
        velMsg.angular.y = 0;
        velMsg.angular.z = turnRate;

        turtleVel.publish(velMsg);

        robotTh += turnRate / 60.0;

        motorL.update(1.0/60.0);
        motorR.update(1.0/60.0);

        // All callbacks are handled here, so no chance of threading issues
        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}
