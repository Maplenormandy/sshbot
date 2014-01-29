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

        self.peeper = Peeper(self.colorCb, debug=True, camera=0)

    def colorCb(self, colour):
# self.screw_queue[self.screw_ind%90] = colour
# self.screw_ind += 1
        self.timer += 1
        if colour == 'R':
            self.screw_cmd.publish(Float32(data=-0.1))
            if self.timer > 2:
                self.kick_cmd.publish(Int16(data=140))
                rospy.loginfo("PUNCH")
            else:
                self.kick_cmd.publish(Int16(data=180))
            
            if self.timer > 10:
                self.kick_cmd.publish(Int16(data=180))
                self.timer = 0
                
        else:
            self.screw_cmd.publish(Float32(data=-0.3))
            if self.timer > 10:
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




