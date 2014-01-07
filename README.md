Smiling Sunshine Happy Bot
==========================

Main repo for Smiling Sunshine Happy Bot.

Should be placed under ~/catkin_ws/src/ after initializing the workspace.

Please add individual readmes to each subdirectory!

The username/password for the tablet is sshb:sshb

# Installing the Maple

This was hard.

First, remove any installations of rosserial

    sudo apt-get purge ros-hydro-rosserial*

The version of rosserial in the debs is old. Then, run this to make the new rosserial package.

    cd ~/catkin_ws
    catkin_make
    sudo catkin_make install

Note that there's a slight change in this version of rosserial from the git version. Namely, the rosserial arduino package was edited for Maple compatibility.

Next, install the Maple IDE and make sure you can run regular maple code. Go to <IDE folder>/libraries and run

    rosrun rosserial_arduino make_libraries.py .

Try and compile any program from the Maple IDE to make sure it's working.

Finally, when you want to put a program on the Maple that uses rosserial, look at the rosserial_arduino tutorials (they'll translate directly). Simply deply from the Maple IDE, then run

    rosrun rosserial_python serial_node.py _port:=/dev/ttyACM# _baud:=57600

where # is the Maple USB serial port.

There you go! Everything should work now.
