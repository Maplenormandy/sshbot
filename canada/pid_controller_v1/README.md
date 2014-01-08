PID Controller v1
=================

Is actually a PD controller. Implements a /cmd_vel and /odom channel for the motors from Maple at 60 Hz. To run, deploy it to the Maple, then run the rosserial_python serial node.

If it's working, then the LED will blink at 60 Hz (which is fast).

To make the robot move forward at 0.2 m/s while turning at 0.1 rad/s (right hand rule), input

    rostopic pub -1 /cmd_vel geometry_msgs/Twist -- '[0.2, 0, 0]' '[0, 0, 0.1]'

There is also a channel to tune the PD loop, /pid_tune. To change the gains, do

    rostopic pub -1 /pid_tune geometry_msgs/Twist -- '[150000, 12000, 3]' '[0.7, 0, 0]'

The first is Kp, the gain proportional to the theta value of the wheels. The second is Kd, the gain proportional to dtheta/dt of the wheels. The last is the maximum windup of the velocity command integrator. The fourth is the low-pass gain filter on the derivative.

There is also a channel /chatter which the robot will post status and error messages to.
