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
        self.globaltimer = 0
        self.ks = 'WAITING_FOR_RED'  
        rospy.loginfo("start");
        self.peeper = Peeper(self.colorCb, debug=True, camera=0)

    def colorCb(self, colour):
# self.screw_queue[self.screw_ind%90] = colour
# self.screw_ind += 1
  

            
        if self.ks == 'WAITING_FOR_RED':
            self.screw_cmd.publish(Float32(data=-0.5))
            self.kick_cmd.publish(Int16(data=180))
            if colour == 'R':
                rospy.loginfo("I SEE A RED PLEASE KICK")
                self.ks = 'DO_A_KICK'
            
            self.globaltimer += 1
            if self.globaltimer > 480: #nothing for 20 seconds
                rospy.loginfo("stuck for 20 seconds, something wrong?")
                self.ks = 'DO_A_CHILL'
                self.globaltimer = 0
        
        if self.ks == 'DO_A_KICK':
            self.timer = 0
            self.ks = 'WAIT_TILL_KICK_DONE'
        
        if self.ks == 'WAIT_TILL_KICK_DONE':
            self.timer += 1
            if self.timer > 10:
                self.ks = 'WAITING_FOR_RED'
            elif self.timer > 6:
                self.screw_cmd.publish(Float32(data=-0.5))
                self.kick_cmd.publish(Int16(data=180))
            elif self.timer > 3:
                self.screw_cmd.publish(Float32(data=-0))
                self.kick_cmd.publish(Int16(data=140))
        
        if self.ks == 'DO_A_CHILL':
            self.timer = 0
            self.screw_cmd.publish(Float32(data=0))
            self.kick_cmd.publish(Int16(data=180))
            self.ks = 'STILL_CHILLING'
            
        if self.ks == 'STILL_CHILLING':
            self.timer += 1
            if self.timer > 24:
                self.ks = 'WAITING_FOR_RED'
            elif self.timer > 12:
                self.screw_cmd.publish(Float32(data=0.4))
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




