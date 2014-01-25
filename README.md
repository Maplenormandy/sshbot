Smiling Sunshine Happy Bot
==========================

Main repo for Smiling Sunshine Happy Bot.

Should be placed under ~/catkin_ws/src/ after initializing the workspace.

Please add individual readmes to each subdirectory!

The username/password for the tablet is sshb:sshb

# How to launch our robot, The Definitive Guide

This guide is up to date as of 1/25.

First, since I accidentally updated ROS (really sorry guys!), run

    sudo apt-get update
    sudo apt-get upgrade

There may be errors with the upgrade. If there are, follow the recommended instructions. I think I had to do something along the lines of

    sudo apt-get -f install

To run the bot, in separate terminals run:

    roscore
    roslaunch b2b bridge.launch

To run the alignment code, run:

    roslaunch paralympics align_reactor.launch

To run gmapping (SLAM), run:

    roslaunch nav gmap.launch

To run AMCL (localization). Please note this may not actually work as the right IRs are not working. TODO would be to fix msg_parser.cpp in b2b and remove the right IR data and hope it works.

    roslaunch nav move_base.launch

Please note that these three launch files are currently mutually exclusive.

If you want to run on a simulated robot, run

    rosrun b2b pretend_bot.py

Instead of running bridge.launch.

If you want to visualize the odometry/pose of the robot, run

    rosrun rviz rviz

Then add a TF display. Fix it to the map frame to see where the robot thinks it is, or fix it to the odom frame to see where the robot's odometry reports it as being.

# Installing the Maple

This was hard.

First, remove any installations of rosserial

    sudo apt-get purge ros-hydro-rosserial*

The version of rosserial in the debs is old. Then, go to https://github.com/ros-drivers/rosserial and follow the instructions under Usage/Workflow. However, before running catkin_make, look for the rosserial_arudino.zip file in the canada folder. Unzip this over the rosserial_arduino folder in the rosserial repo.

Note that there's a slight change in this version of rosserial from the git version. Namely, the rosserial arduino package was edited for Maple compatibility.

Next, install the Maple IDE and make sure you can run regular maple code. Go to <IDE folder>/libraries and run

    rosrun rosserial_arduino make_libraries.py .

Try and compile any program from the Maple IDE to make sure it's working.

Finally, when you want to put a program on the Maple that uses rosserial, look at the rosserial_arduino tutorials (they'll translate directly). Simply deply from the Maple IDE, then run

    rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=115200

where /dev/ttyACM0 is the Maple USB serial port.

There you go! Everything should work now.

# ROS Packages Used

Actual:
* rosserial
* smach
* numpy/scipy
* OpenCV

Proposed:
* navigation
    * LaserScan
    * Odometry
* tf
