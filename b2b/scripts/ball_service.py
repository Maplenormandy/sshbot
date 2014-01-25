#!/usr/bin/env python
import roslib; roslib.load_manifest('b2b')
import rospy
from std_msgs.msg import String, UInt16, Float32
import math
import numpy as np
import thread
import threading
from b2b.srv import *

class BallHandler():
    def __init__(self):
        self.green_tray = 0
        self.green_queued = 0

        self.kicker_pub = rospy.Publisher('/kick_cmd', UInt16)
        self.pac_pub = rospy.Publisher('/pac_cmd', UInt16)
        self.gate_g_pub = rospy.Publisher('/gate_g_cmd', UInt16)
        self.gate_r_pub = rospy.Publisher('/gate_r_cmd', UInt16)

        self.screw_sub = rospy.Subscriber('/profit/screw', String, self.saw_ball)

        self.ball_lock = threading.RLock()
        self.ball_srv = rospy.Service('green_ball_server', GreenBallService, self.greenBallSrv)
        self.ball_dump = rospy.Service('ball_dump', BallDump, self.ballDump)

    def kickGreenBall(self):
        rospy.loginfo('kicking ball')
        msg = UInt16()
        rospy.sleep(1.0)
        msg.data = 90
        self.kicker_pub.publish(msg)
        rospy.sleep(1.0)
        msg.data = 0
        self.kicker_pub.publish(msg)
        rospy.sleep(1.0)
        self.ball_lock.acquire()
        self.green_tray += 1
        self.ball_lock.release()

    def ballDump(self, req):
        rospy.loginfo('dumping ' + req.color)
        resp = BallDumpResponse()
        msg = UInt16()
        msg.data = 180

        if req.color=='g':
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

            msg = UInt16()
            msg.data = 180
            self.pac_pub.publish(msg.data)
            rospy.sleep(0.4)

            self.ball_lock.acquire()
            self.green_tray -= 1
            self.green_queued += 1
            self.ball_lock.release()

            msg.data = 0
            self.pac_pub.publish(msg.data)
            rospy.sleep(0.4)

        resp.tray = self.green_tray
        resp.queued = self.green_queued

        return resp

    def saw_ball(self, msg):
        if msg.data == 'g':
            rospy.loginfo('saw green ball')
            thread.start_new_thread(self.kickGreenBall, ())
        elif msg.data == 'r':
            rospy.loginfo('saw red ball')



def main():
    rospy.init_node('ball_service')
    bh = BallHandler()
    rospy.spin()

if __name__=='__main__':
    main()
