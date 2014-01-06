kelp
====

Smiling Sunshine Happy Bot turtlesim project. Adds dynamics on top of the turtlesim in ROS so we can test simple drive control algorithms and such.

Note, this is only compatible with the latest version of ROS (Hydro, Ubuntu 13.04) on account of turtlesim changing between those two versions.

Make sure you clone this into your catkin workspace, so kelp is the src folder.

Then test it works with

    roscd kelp

If it all works then you should be able to build and run kelp.

# Building

Go to the base catkin_ws directory and make:

    catkin_make

# Launching

There's a launch configuration setup that sets up the parameters used by turtle_motor called turtle_motor.launch. Thus, to run kelp after building it simply type

    roslaunch kelp turtle_motor.launch

The parameters defined in the .launch file define the motor properties and turtle robot properties.

# Running

turtle_motor subscribes to two std_msgs/Float32 topics, "voltage_left" and "voltage_right". It publishes two std_msgs/Float32 topics, "enc_left" and "enc_right".
