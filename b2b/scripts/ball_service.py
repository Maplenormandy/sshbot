#!/usr/bin/env python
import roslib; roslib.load_manifest('b2b')
import rospy
from std_msgs.msg import String, Int16, Float32
import math
import numpy as np
import thread
import threading
from b2b.srv import *
import threading


class BallHandler():
    def __init__(self):
        self.green_tray = 999
        self.green_queued = 0

        self.roller_pub = rospy.Publisher('/roller_cmd', Float32)
        #self.roller_pub.publish(Float32(data=0.5), latch=True)

        self.screw_queue = ['']*90
        self.screw_ind = 0

        self.kick_pub = rospy.Publisher('/kick_cmd', Int16)
        self.pac_pub = rospy.Publisher('/pac_cmd', Int16)
        self.pac_pub.publish(Int16(data=70))
        self.gate_g_pub = rospy.Publisher('/gate_g_cmd', Int16)
        self.gate_r_pub = rospy.Publisher('/gate_r_cmd', Int16)
        self.gate_g_pub.publish(Int16(data=0))

        self.screw_sub = rospy.Subscriber('/profit/screw', String, self.saw_ball)

        self.ball_lock = threading.RLock()
        self.ball_srv = rospy.Service('green_ball_server', GreenBallService, self.greenBallSrv)
        self.ball_dump = rospy.Service('ball_dump', BallDump, self.ballDump)

    def ballDump(self, req):
        rospy.loginfo('dumping ' + req.color)
        resp = BallDumpResponse()
        msg = Int16()
        msg.data = 0

        if req.color=='g':
            msg.data = 90
            self.gate_g_pub.publish(msg)
            rospy.sleep(1.0)

            self.ball_lock.acquire()
            resp.dumped = self.green_queued
            self.green_queued = 0
            self.ball_lock.release()

            msg.data = 0
            self.gate_g_pub.publish(msg)
        elif req.color=='r':
            self.gate_r_pub.publish(msg)
            rospy.sleep(1.0)

            resp.dumped = 65535

            msg.data = 0
            self.gate_r_pub.publish(msg)

        return resp


    def greenBallSrv(self, req):
        resp = GreenBallServiceResponse()

        to_queue = max(req.requested-self.green_queued, 0)

        for i in range(to_queue):
            if self.green_tray == 0:
                break

            rospy.loginfo('queueing green ball')

            msg = Int16()
            msg.data = 30
            self.pac_pub.publish(msg.data)
            rospy.sleep(1.0)

            self.ball_lock.acquire()
            self.green_tray -= 1
            self.green_queued += 1
            self.ball_lock.release()

            msg.data = 70
            self.pac_pub.publish(msg.data)
            rospy.sleep(1.0)

        resp.tray = self.green_tray
        resp.queued = self.green_queued

        return resp

    def saw_ball(self, msg):
        self.screw_queue[self.screw_ind] = msg.data
        self.screw_ind += 1
        if self.screw_queue[self.screw_ind-60] == 'g':
            self.kick_cmd.publish(Int16(data=180))
        else:
            self.kick_cmd.publish(Int16(data=0))


def main():
    rospy.init_node('ball_service')
    bh = BallHandler()
    rospy.spin()

if __name__=='__main__':
    main()
