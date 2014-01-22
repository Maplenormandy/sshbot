#!/usr/bin/env python
import roslib; roslib.load_manifest('paralympics')
import rospy
from geometry_msgs.msg import Twist, PointStamped, Pose
from std_msgs.msg import UInt16
from actionlib_tutorials.msg import FibonacciAction
import math
import numpy as np
from sensor_state import SensorState
from init_state import InitState
from wallflower import Wallflower
from profit.msg import BallArray
from smach import *
from smach_ros import *

__all__ = ['DispenserState']


class AlignToSilo(SensorState):
    def __init__(self, cmd_vel_pub):
        SensorState.__init__(self, '/profit/wall_raw', PointStamped, 0.1,
                outcomes=['succeeded', 'preempted', 'aborted'])

        self._cmd_vel = cmd_vel_pub

    def execute(self, ud):
        return 'succeeded'

class GrabSiloBalls(State):
    def __init__(self, sas_pub):
        State.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'])
        self._sas_pub = sas_pub

    def execute(self, ud):
        msg = UInt16()
        msg.data = 180
        self._sas_pub.publish(msg)
        rospy.sleep(0.5)
        msg.data = 0
        self._sas_pub.publish(msg)

        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'

        return 'succeeded'

    def request_preempt(self):
        msg = UInt16()
        msg.data = 0
        self._sas_pub.publish(msg)
        State.request_preempt(self)

""" Eventually, want to make sure that there are balls in the silo
class CheckBallsFromSilo(SensorState):
    def __init__(self):
        SensorState.__init__(self, '/profit/ball_silo_raw',
"""

# TODO remove hack here
ballsColl = 0

class CheckSiloBalls(State):
    def __init__(self):
        State.__init__(self, outcomes=['valid', 'invalid', 'preempted'])

    def execute(self, ud):
        global ballsColl
        ballsColl += 1
        rospy.sleep(2.0)
        if ballsColl < 5:
            return 'valid'
        else:
            return 'invalid'

class DispenserState(StateMachine):
    def __init__(self):
        StateMachine.__init__(self,
                outcomes=['succeeded', 'preempted', 'aborted'])

        sas_pub = rospy.Publisher('/sas_cmd', UInt16)
        cmd_vel = rospy.Publisher('/cmd_vel', Twist)

        with self:
            StateMachine.add('ALIGN_SILO', AlignToSilo(cmd_vel),
                    transitions={'succeeded':'CHECK_SILO'})
            StateMachine.add('CHECK_SILO', CheckSiloBalls(),
                    transitions={'valid':'GRAB_SILO',
                        'invalid':'succeeded'})
            StateMachine.add('GRAB_SILO', GrabSiloBalls(sas_pub),
                    transitions={'succeeded':'CHECK_SILO',
                        'aborted':'ALIGN_SILO'})

def main():
    rospy.init_node('disptest')
    sm_disp = DispenserState()
    sm_disp.execute()

if __name__=='__main__':
    main()
