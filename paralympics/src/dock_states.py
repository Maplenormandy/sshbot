#!/usr/bin/env python
import roslib; roslib.load_manifest('paralympics')
import rospy
from geometry_msgs.msg import Twist, PointStamped, Pose
from std_msgs.msg import Int16
from actionlib_tutorials.msg import FibonacciAction
import math
import numpy as np
from sensor_state import SensorState
from profit.msg import BallArray
from smach import *
from smach_ros import *
from b2b.srv import *

__all__ = ['DispenserState']


class AlignToReactor(SensorState):
    def __init__(self, cmd_vel_pub):
        SensorState.__init__(self, '/profit/reactor_wall_raw', PointStamped, 0.1,
                input_keys=['desired_dist'],
                outcomes=['succeeded', 'preempted', 'aborted']
                )

        self._cmd_vel = cmd_vel_pub

    def execute(self, ud):
        return 'succeeded'

class QueueGreenBalls(State):
    def __init__(self):
        State.__init__(self,
                input_keys=['requested'],
                output_keys=['queued'],
                outcomes=['succeeded', 'preempted', 'aborted']
                )

        self.green_ball_srv = rospy.ServiceProxy('green_ball_server', GreenBallService)

    def execute(self, ud):
        ud.queued = self.green_ball_srv(ud.requested).queued
        return 'succeeded'

class DumpGreenBalls(State):
    def __init__(self):
        State.__init__(self,
                output_keys=['dumped'],
                outcomes=['succeeded', 'preempted', 'aborted']
                )

        self.ball_dump = rospy.ServiceProxy('ball_dump', BallDump)

    def execute(self, ud):
        ud.dumped = self.ball_dump('g').dumped
        return 'succeeded'

class ReactorState(StateMachine):
    def __init__(self):
        StateMachine.__init__(self,
                input_keys=['high_balls', 'low_balls'],
                outcomes=['succeeded', 'preempted', 'aborted']
                )

        self._cmd_vel = rospy.Publisher('/cmd_vel', Twist)

        self.sm_high = AlignAndQueue(self._cmd_vel)
        self.sm_low = AlignAndQueue(self._cmd_vel)

        self.userdata.high_dist = 0.1
        self.userdata.low_dist = 1.0

        with self:
            StateMachine.add('SM_HIGH', self.sm_high,
                    remapping={
                        'balls':'high_balls',
                        'dist':'high_dist'
                        },
                    transitions={'succeeded':'DUMP_HIGH'}
                    )
            StateMachine.add('DUMP_HIGH', DumpGreenBalls(),
                    transitions={'succeeded':'SM_LOW'}
                    )
            StateMachine.add('SM_LOW', self.sm_low,
                    remapping={
                        'balls':'low_balls',
                        'dist':'low_dist',
                        },
                    transitions={'succeeded':'DUMP_LOW'}
                    )
            StateMachine.add('DUMP_LOW', DumpGreenBalls())

class AlignAndQueue(Concurrence):
    def __init__(self, cmd_vel_pub):
        Concurrence.__init__(self,
                input_keys=['dist', 'balls'],
                output_keys=['queued'],
                outcomes=['succeeded', 'preempted', 'aborted'],
                default_outcome='aborted',
                child_termination_cb = self.child_termination_cb,
                outcome_cb = self.outcome_cb
                )
        self._cmd_vel = cmd_vel_pub

        with self:
            Concurrence.add('ALIGN', AlignToReactor(self._cmd_vel),
                    remapping={'desired_dist':'dist'}
                    )
            Concurrence.add('QUEUE', QueueGreenBalls(),
                    remapping={'requested':'balls'}
                    )


    def child_termination_cb(self, outmap):
        if all(map(lambda x: x=='succeeded', outmap.values())):
            return True
        elif any(map(lambda x: x=='aborted' or x=='preempted', outmap.values())):
            return True
        else:
            return False

    def outcome_cb(self, outmap):
        if all(map(lambda x: x=='succeeded', outmap.values())):
            return 'succeeded'
        elif any(map(lambda x: x=='preempted', outmap.values())):
            return 'preempted'
        else:
            return 'aborted'



class AlignToSilo(SensorState):
    def __init__(self, cmd_vel_pub):
        SensorState.__init__(self, '/profit/silo_wall_raw', PointStamped, 0.1,
                outcomes=['succeeded', 'preempted', 'aborted'])

        self._cmd_vel = cmd_vel_pub

    def execute(self, ud):
        return 'succeeded'

class GrabSiloBalls(State):
    def __init__(self, sas_pub):
        State.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'])
        self._sas_pub = sas_pub

    def execute(self, ud):
        msg = Float32()
        msg.data = 0.5
        self._sas_pub.publish(msg)
        rospy.sleep(0.5)
        msg.data = -0.5
        self._sas_pub.publish(msg)
        rospy.sleep(0.5)
        msg.data = 0.0
        self._sas_pub.publish(msg)

        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'

        return 'succeeded'

    def request_preempt(self):
        msg = Int16()
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

        sas_pub = rospy.Publisher('/sas_cmd', Int16)
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
    rospy.init_node('docktest')
    #sm_disp = DispenserState()
    #sm_disp.execute()
    sm_reactor = ReactorState()
    sm_reactor.userdata.high_balls = 4
    sm_reactor.userdata.low_balls = 1
    sm_reactor.execute()

if __name__=='__main__':
    main()
