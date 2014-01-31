#!/usr/bin/env python
import subprocess
import os
import signal
import time
import shlex



process1 = subprocess.Popen(
    shlex.split("""x-terminal-emulator -e 'bash -c "roscore"'"""), stdout=subprocess.PIPE)

process2 = subprocess.Popen(
    shlex.split("""x-terminal-emulator -e 'bash -c "rosrun profit setcams.py"'"""), stdout=subprocess.PIPE)

process3 = subprocess.Popen(
    shlex.split("""x-terminal-emulator -e 'bash -c "botclient_test_server/Python/botclient.py"'"""), stdout=subprocess.PIPE, preexec_fn=os.setsid)

process4 = subprocess.Popen(
    shlex.split("""x-terminal-emulator -e 'bash -c "cd ~;rosrun nav parse_map.py"'"""), stdout=subprocess.PIPE)

process5 = subprocess.Popen(
    shlex.split("""x-terminal-emulator -e 'bash -c "rosrun profit ros_ballseeingeye.py"'"""), stdout=subprocess.PIPE)

process6 = subprocess.Popen(
    shlex.split("""x-terminal-emulator -e 'bash -c "rosrun profit peepros.py"'"""), stdout=subprocess.PIPE)

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
