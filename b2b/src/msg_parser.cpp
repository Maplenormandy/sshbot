#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/TwistStamped.h>
#include <b2b/IRStamped.h>
#include <boost/math/constants/constants.hpp>
#include <boost/assign/list_of.hpp>
#include <cmath>
#include <limits>

ros::Publisher* p_odom_pub;
tf::TransformBroadcaster* p_odom_broadcaster;

const double pi = boost::math::constants::pi<double>();
const float irRange = 0.45f;
const float dist_to_ir_center = 0.0513842f;
const float dist_to_ir_center_fwd = 0.2f;

nav_msgs::Odometry odom;

void futzOdom(const geometry_msgs::TwistStamped& msg)
{
    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat =
        tf::createQuaternionMsgFromYaw(msg.twist.linear.z);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = msg.header.stamp;
    odom_trans.header.seq = msg.header.seq;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    // Get transformation from odom coordinates. Note that the odom_partial
    // message is strange.
    odom_trans.transform.translation.x = msg.twist.angular.x;
    odom_trans.transform.translation.y = msg.twist.angular.y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    p_odom_broadcaster->sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    odom.header.stamp = msg.header.stamp;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = msg.twist.angular.x;
    odom.pose.pose.position.y = msg.twist.angular.y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = msg.twist.linear.x;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = msg.twist.angular.z;

    //publish the message
    p_odom_pub->publish(odom);
}

struct irReading
{
    float l;
    static const float lowpass = 0.4f;
    const float minRange;
    const float maxRange;

    irReading(float min, float max) : minRange(min), maxRange(max)
    {
        l = (minRange+maxRange)/2.0f;
    }

    float update(float in)
    {
        if (in < minRange)
        {
            l = minRange*lowpass + l*(1-lowpass);
            return -std::numeric_limits<float>::infinity();
        }
        else if (in > maxRange)
        {
            l = maxRange*lowpass + l*(1-lowpass);
            return std::numeric_limits<float>::infinity(); 
        }
        else
        {
            l = in*lowpass + l*(1-lowpass);
            return l;
        }
    }
};

struct irFutzer
{
    ros::Publisher lscan_pub, rscan_pub, flscan_pub, frscan_pub, fscan_pub, scan_pub;
    sensor_msgs::LaserScan lscan, rscan, flscan, frscan, fscan;


    irReading lfwd, lmid, lbak, rfwd, rmid, rbak, fwdl, fwdr;

    ros::NodeHandle n;

    irFutzer() : 
        lfwd(0.1+dist_to_ir_center, irRange+dist_to_ir_center),
        lmid(0.1+dist_to_ir_center, irRange+dist_to_ir_center),
        lbak(0.1+dist_to_ir_center, irRange+dist_to_ir_center),
        rfwd(0.1+dist_to_ir_center, irRange+dist_to_ir_center),
        rmid(0.1+dist_to_ir_center, irRange+dist_to_ir_center),
        rbak(0.1+dist_to_ir_center, irRange+dist_to_ir_center),
        fwdl(0.04, 0.22),
        fwdr(0.04, 0.22)
    {
        lscan_pub = n.advertise<sensor_msgs::LaserScan>("lscan", 150);
        rscan_pub = n.advertise<sensor_msgs::LaserScan>("rscan", 150);
        flscan_pub = n.advertise<sensor_msgs::LaserScan>("flscan", 150);
        frscan_pub = n.advertise<sensor_msgs::LaserScan>("frscan", 150);
        fscan_pub = n.advertise<sensor_msgs::LaserScan>("fscan", 150);
        scan_pub = n.advertise<sensor_msgs::LaserScan>("scan", 150);

        frscan.header.frame_id = "frscan";
        frscan.angle_min = 0.0;
        frscan.angle_max = 0.1;
        frscan.angle_increment = 0.1;
        frscan.time_increment = 0.0;
        frscan.scan_time = .045;
        frscan.range_min = 0.04;
        frscan.range_max = 0.22;
        frscan.ranges.resize(1);

        flscan.header.frame_id = "flscan";
        flscan.angle_min = 0.0;
        flscan.angle_max = 0.1;
        flscan.angle_increment = 0.1;
        flscan.time_increment = 0.0;
        flscan.scan_time = .045;
        flscan.range_min = 0.04;
        flscan.range_max = 0.22;
        flscan.ranges.resize(1);

        fscan.header.frame_id = "fscan";
        fscan.angle_min = -0.1;
        fscan.angle_max = 0.1;
        fscan.angle_increment = 0.2;
        fscan.time_increment = 0.0;
        fscan.scan_time = .045;
        fscan.range_min = 0.04+dist_to_ir_center_fwd;
        fscan.range_max = 0.22+dist_to_ir_center_fwd;
        fscan.ranges.resize(2);

        lscan.header.frame_id = "lscan";
        lscan.angle_min = -pi/4.0;
        lscan.angle_max = pi/4.0;
        lscan.angle_increment = pi/4.0;
        lscan.time_increment = 0;
        lscan.scan_time = .045;
        lscan.range_min = dist_to_ir_center + .1;
        lscan.range_max = dist_to_ir_center + irRange;
        lscan.ranges.resize(3);

        rscan.header.frame_id = "rscan";
        rscan.angle_min = -pi/4.0;
        rscan.angle_max = pi/4.0;
        rscan.angle_increment = pi/4.0;
        rscan.time_increment = 0;
        rscan.scan_time = .045;
        rscan.range_min = dist_to_ir_center + .1;
        rscan.range_max = dist_to_ir_center + irRange;
        rscan.ranges.resize(3);
    }

    void irToLaser(const b2b::IRStamped &msg)
    {

        float fr = fwdr.update(msg.fwd_r);
        //=======Front Right IR========
        frscan.header.stamp = msg.header.stamp;
        frscan.header.seq = msg.header.seq;
        frscan.ranges[0] = fr;
        //publish the message
        frscan_pub.publish(frscan);

        float fl = fwdl.update(msg.fwd_l);
        //=======Front Left IR========
        flscan.header.stamp = msg.header.stamp;
        flscan.header.seq = msg.header.seq;
        flscan.ranges[0] = fl;
        //publish the message
        flscan_pub.publish(flscan);

        //=======Front IR========
        fscan.header.stamp = msg.header.stamp;
        fscan.header.seq = msg.header.seq;
        fscan.ranges[0] = fl+dist_to_ir_center_fwd;
        fscan.ranges[1] = fr+dist_to_ir_center_fwd;
        //publish the message
        fscan_pub.publish(fscan); 

        //=======Left IR========
        lscan.header.stamp = msg.header.stamp;
        lscan.header.seq = msg.header.seq;
        lscan.ranges[0] = lfwd.update(msg.l.fwd + dist_to_ir_center);
        lscan.ranges[1] = lmid.update(msg.l.mid + dist_to_ir_center);
        lscan.ranges[2] = lbak.update(msg.l.bak + dist_to_ir_center);
        //publish the message
        lscan_pub.publish(lscan);

        //=======Right IR========
        rscan.header.stamp = msg.header.stamp;
        rscan.header.seq = msg.header.seq;
        rscan.ranges[2] = rfwd.update(msg.r.fwd + dist_to_ir_center);
        rscan.ranges[1] = rmid.update(msg.r.mid + dist_to_ir_center);
        rscan.ranges[0] = rbak.update(msg.r.bak + dist_to_ir_center);
        //publish the message
        rscan_pub.publish(rscan);

        //scan_pub.publish(frscan);
        //scan_pub.publish(flscan);
        scan_pub.publish(fscan);
        scan_pub.publish(rscan);
        scan_pub.publish(lscan);
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "msg_parser");

    ros::NodeHandle n;

    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    tf::TransformBroadcaster odom_broadcaster;

    p_odom_pub = &odom_pub;
    p_odom_broadcaster = &odom_broadcaster;

    irFutzer futz;

    ros::Subscriber odom_read = n.subscribe("odom_partial", 1000, futzOdom);
    ros::Subscriber scan_read = n.subscribe("ir_raw", 1000, &irFutzer::irToLaser, &futz);

    odom.pose.covariance = boost::assign::list_of
        (1e-3)      (0)     (0)     (0)     (0)     (0)
        (0)      (1e-3)     (0)     (0)     (0)     (0)
        (0)      (0)     (1e6)     (0)     (0)     (0)
        (0)      (0)     (0)     (1e6)     (0)     (0)
        (0)      (0)     (0)     (0)     (1e6)     (0)
        (0)      (0)     (0)     (0)     (0)     (1);

    odom.twist.covariance = boost::assign::list_of
        (1e-3)      (0)     (0)     (0)     (0)     (0)
        (0)      (1e-3)     (0)     (0)     (0)     (0)
        (0)      (0)     (1e6)     (0)     (0)     (0)
        (0)      (0)     (0)     (1e6)     (0)     (0)
        (0)      (0)     (0)     (0)     (1e6)     (0)
        (0)      (0)     (0)     (0)     (0)     (1);


    ros::spin();
}

