#!/usr/bin/env python
import roslib; roslib.load_manifest('profit')
import rospy
import numpy as np
from peeper import Peeper
from std_msgs.msg import Int16

class Peepros:
    def __init__(self):
        self.rate = rospy.Rate(16)
        self.kick_cmd = rospy.Publisher('/kick_cmd', Int16)

        self.screw_queue = ['N']*90
        self.screw_ind = 0

        self.peeper = Peeper(self.colorCb, camera=2)

    def colorCb(self, colour):
        self.screw_queue[self.screw_ind%90] = colour
        self.screw_ind += 1
        if self.screw_queue[(self.screw_ind)%90] == 'R':
            self.kick_cmd.publish(Int16(data=140))
        else:
            self.kick_cmd.publish(Int16(data=180))

        rospy.loginfo(colour)

    def execute(self):
        while not rospy.is_shutdown():
            self.peeper.loop()
            self.rate.sleep()


def main():
    rospy.init_node('peeper')
    pr = Peepros()
    pr.execute()


if __name__=='__main__':
    main()




