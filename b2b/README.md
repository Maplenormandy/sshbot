Bridge To Bot (B2B)
===================

Does several helpful utility things for connecting the maple and the computer. TO run, simply use

    roslaunch b2b bridge.launch

Current features are:

* Parses odometry and laserscan data from maple
* Keeps track of green balls and actuates sorter

Planned features:

* Set PID gains

# Channels

Published:
* odom_partial (geometry_msgs/TwistStamped)
* ir_raw (b2b/IRStamped)
* chatter (std_msgs/String)
Subscribed:
* cmd_vel (geometry_msgs/Twist)
* sas_cmd (std_msgs/UInt16)
* kick_cmd (std_msgs/UInt16)
* pac_cmd (std_msgs/UInt16)
* gate_g_cmd (std_msgs/UInt16)
* gate_r_cmd (std_msgs/UInt16)
* roll_cmd (std_msgs/Float32)
* screw_cmd (std_msgs/Float32)
