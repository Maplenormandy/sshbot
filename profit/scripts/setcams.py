#!/usr/bin/env python
import rospy
import subprocess

def main():
    for i in range(4):
        try:
            subprocess.call(["v4l2-ctl",
                "-d", "/dev/video" + str(i),
                "-c", "white_balance_temperature_auto=0"])
            rospy.sleep(0.1)
            subprocess.call(["v4l2-ctl",
                "-d", "/dev/video" + str(i),
                "-c", "white_balance_temperature=3100"])
            rospy.sleep(0.1)
            print subprocess.check_output(["v4l2-ctl",
                "-d", "/dev/video" + str(i),
                "-l"])
        except:
            print "No camera " + str(i)


if __name__=='__main__':
    main()

