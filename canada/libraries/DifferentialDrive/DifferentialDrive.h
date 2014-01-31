#ifndef DIFFERENTIAL_DRIVE_H_
#define DIFFERENTIAL_DRIVE_H_

#include <Encoder.h>
#include <SensorTimer.h>
#include <PinSettings.h>

#include <stdlib.h>
#include <ros.h>

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"
#include "std_msgs/Empty.h"

class DifferentialDrive
{
    public:
        DifferentialDrive();

        geometry_msgs::TwistStamped odom_msg;
        std_msgs::Empty overspeed_msg;
        static geometry_msgs::Twist cmd_vel_msg;

        ros::Publisher odom;
        ros::Publisher overspeed;

        ros::Subscriber<geometry_msgs::Twist> pid_tune, cmd_vel;

        // Gains, see the cpp file for values
        static double kp, kd;
        static double lowpass;
        // Saturation for integration
        static double int_sat;

        static void pidTuneCb(const geometry_msgs::Twist& msg);
        static void cmdCb(const geometry_msgs::Twist& msg);

        /*
         * y - measured value
         */
        // Distance values
        double yl;
        double yr;
        // Integrated distance values
        double yil;
        double yir;
        // Last distance values
        double yll;
        double ylr;
        // Last filtered derivative value
        double ydll;
        double ydlr;

        /*
         * r - signal value
         */
        // Signal value
        double rdl;
        double rdr;
        // Integrated signal value
        double rl;
        double rr;
        // Last signal value
        double rll;
        double rlr;

        /*
         * Pose
         */
        double th;
        double x;
        double y;

        /*
         * Odometry rate limiter
         */
        int odomSeq;

        /*
         * Robot Parameters
         */
        // 3 7/8" diam wheels with a fudge factor
        static const double wheelR = 3.875*2.54/200.0 / 1.05;
        // halfway across bot in meters
        static const double axleR = 10.75*2.54/200.0;
        // encoder ticks to theta
        static const double convFactor = 2.0*PI/64.0/29.0; 

        // Encoders from encoder library
        Encoder encL, encR;

        // SensorTimer, useful helper class
        SensorTimer st;

        /*
         * Functions
         */
        void reset(void);
        void loop(void);
};

#endif
