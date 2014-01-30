#!/usr/bin/env python
import roslib; roslib.load_manifest('profit')
import rospy
import numpy as np
from peeper import Peeper
from std_msgs.msg import Int16, Float32

class Peepros:
    def __init__(self):
        self.rate = rospy.Rate(24)
        self.kick_cmd = rospy.Publisher('/kick_cmd', Int16)
        self.screw_cmd = rospy.Publisher('/screw_cmd', Float32)

# self.screw_queue = ['N']*90
# self.screw_ind = 0
        self.timer = 0
        self.ks = 'WAITING_FOR_RED'  
        self.peeper = Peeper(self.colorCb, debug=True, camera=0)

    def colorCb(self, colour):
# self.screw_queue[self.screw_ind%90] = colour
# self.screw_ind += 1

        if self.ks == 'WAITING_FOR_RED':
            self.screw_cmd.publish(Float32(data=-0.3))
            self.kick_cmd.publish(Int16(data=180))
            if colour == 'R':
                self.ks = 'DO_A_KICK'
        
        if self.ks == 'DO_A_KICK':
            self.timer = 0
            self.screw_cmd.publish(Float32(data=-0))
            self.kick_cmd.publish(Int16(data=140))
            self.ks = 'WAIT_TILL_KICK_DONE'
        
        if self.ks == 'WAIT_TILL_KICK_DONE':
            self.timer += 1
            if self.timer > 6:
                self.ks = 'WAITING_FOR_RED'
            elif self.timer > 4:
                self.screw_cmd.publish(Float32(data=-0.3))
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




