Smiling Sunshine Happy Bot
==========================

Main repo for Smiling Sunshine Happy Bot.

Should be placed under ~/catkin_ws/src/ after initializing the workspace.

Please add individual readmes to each subdirectory!

The username/password for the tablet is sshb:sshb

# How to launch our robot, The Definitive Guide

This guide is up to date as of 1/28.

First, since I accidentally updated ROS (really sorry guys!), run

    sudo apt-get update
    sudo apt-get upgrade

There may be errors with the upgrade. If there are, follow the recommended instructions. I think I had to do something along the lines of

    sudo apt-get -f install

To run the bot, in separate terminals run:

    roscore
    roslaunch b2b bridge.launch
    python ~/catkin_ws/src/botclient_test_server/Python/botclient.py  #the Botclient server, which will send the mapString to parse_map.py
    rosrun nav parse_map.py #MUST RUN FROM HOME DIRECTORY
    rosrun profit ros_ballseeingeye.py

Make sure you turn on the robot at this point

To start the nav stack:

    roslaunch nav move_base.launch
    roslaunch nav amcl_diff.launch

Alternatively, to run a fake bot, run this instead:

    roscore
    roslaunch b2b pretend.launch
    rosrun nav parse_map.py
    roslaunch nav fake.launch
    rosrun profit ros_ballseeingeye.py

To run the whole state machine, do

    rosrun paralympics dock_states.py

To run just the travel state (look at the main() function to see how to test individual states)

    rosrun paralympics travel_states.py

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
