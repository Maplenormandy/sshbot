#!/usr/bin/env python
import subprocess
import os
import signal
import time
import shlex


process1 = subprocess.Popen(
    shlex.split("""x-terminal-emulator -e 'bash -c "roslaunch nav move_base.launch"'"""), stdout=subprocess.PIPE)

process2 = subprocess.Popen(
    shlex.split("""x-terminal-emulator -e 'bash -c "rosrun paralympics supersonic.py"'"""), stdout=subprocess.PIPE)

process1.wait()

#c = ['sleep', '20']

#handle = Popen(c, stdin=PIPE, stderr=PIPE, stdout=PIPE, shell=True)

#os.system("echo 'hello world'")
#os.system("echo 'hello world'") roscore &
#roslaunch b2b bridge.launch &
#cd ~
#python ~/catkin_ws/src/botclient_test_server/Python/botclient.py &
#rosrun nav parse_map.py &
#rosrun profit ros_ballseeingeye.py
#echo "Yay it runs!" 
