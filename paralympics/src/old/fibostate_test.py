#!/usr/bin/env python
import roslib; roslib.load_manifest('paralympics')
import rospy
from geometry_msgs.msg import Twist, TwistStamped
import math
import numpy as np
from sensor_state import SensorState
from wallflower import Wallflower
from profit.msg import BallArray
from smach import *
from smach_ros import *
from actionlib_tutorials.msg import FibonacciAction

class FiboState(SimpleActionState):
    def __init__(self):
        SimpleActionState.__init__(self,
                'fibonacci',
                FibonacciAction,
                goal_slots=['order'],
                result_slots=['sequence']
                )

    def _goal_feedback_cb(self, feedback):
        rospy.loginfo(feedback.sequence)


def main():
    rospy.init_node('fibotest')
    sm_root = StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])

    with sm_root:
        sm_root.userdata.sm_order = 5
        StateMachine.add('FIBO',
                FiboState(),
                remapping={'order':'sm_order',
                           'sequence': 'sm_seq'}
                )

    sm_root.execute()
    print sm_root.userdata.sm_seq


main()


