#!/usr/bin/env python
import roslib; roslib.load_manifest('paralympics')
import rospy
from geometry_msgs.msg import Twist, Pose
from profit.msg import *
from std_msgs.msg import UInt16MultiArray
from move_base_msgs.msg import *
from actionlib_tutorials.msg import *
import threading
import random
import math
import numpy as np
from sensor_state import SensorState
from profit.msg import BallArray
from smach import *
from smach_ros import *
from actionlib.simple_action_client import SimpleActionClient, GoalStatus

__all__ = ['TravelState']


class BallTracker(SensorState):
    def __init__(self, move_state, chaser_state):
        SensorState.__init__(self, '/profit/balls', UInt16MultiArray, 0.2,
                input_keys=['move_state', 'chaser_state'],
                outcomes=['preempted']
                )

        self.chaser_state = chaser_state
        self.move_state = move_state

    def loop(self, msg, ud):
        self.chaser_state.balls = list(msg.data)
        # TODO Fancier ball sorting and stuff
        self.chaser_state.balls.sort(key=lambda x: -x)
        self.move_state.found_balls()
        self.chaser_state.reset_ball_goal()


class BallChaser(State):
    def __init__(self, action_client, timeout):
        State.__init__(self,
                outcomes=['succeeded', 'no_more_balls', 'preempted', 'aborted']
                )

        self.balls = []

        self.action_client = action_client

        self._timeout = timeout
        self._trigger_cond = threading.Condition()

        self.goal = FibonacciGoal()


    def execute(self, ud):
        while True:
            self._trigger_cond.acquire()
            self._trigger_cond.wait(self._timeout)
            self._trigger_cond.release()

            if self.goal_status == GoalStatus.SUCCEEDED:
                if len(self.balls)>0:
                    self.reset_ball_goal()
                    return 'succeeded'
                else:
                    return 'no_more_balls'

            elif self.goal_status == GoalStatus.PREEMPTED or self.preempt_requested():
                self.action_client.cancel_goal()
                self.service_preempt()
                return 'preempted'
            elif self.goal_status != None:
                return 'aborted'

    def reset_ball_goal(self):
        if len(self.balls)>0:
            rospy.loginfo(self.balls)
            self.goal_status = None
            self.goal.order = self.balls.pop()
            self.action_client.cancel_goal()
            self.action_client.send_goal(self.goal, self.done_cb)

    def done_cb(self, term_state, result):
        self.goal_status = term_state

        self._trigger_cond.acquire()
        self._trigger_cond.notify()
        self._trigger_cond.release()


class Move(State):
    def __init__(self, action_client, timeout):
        State.__init__(self,
                input_keys=['target_pose'],
                outcomes=['succeeded', 'balls_found', 'preempted', 'aborted']
                )

        self.action_client = action_client

        self._timeout = timeout
        self._trigger_cond = threading.Condition()

        self.goal = FibonacciGoal()


    def execute(self, ud):
        self.goal_status = None
        self.goal.order = ud.target_pose

        self.action_client.send_goal(self.goal, self.done_cb)

        self.ball_status = False

        while True:
            self._trigger_cond.acquire()
            self._trigger_cond.wait(self._timeout)
            self._trigger_cond.release()

            if self.goal_status == GoalStatus.SUCCEEDED:
                return 'succeeded'
            elif self.goal_status == GoalStatus.PREEMPTED or self.preempt_requested():
                self.action_client.cancel_goal()
                self.service_preempt()
                return 'preempted'
            elif self.ball_status == True:
                return 'balls_found'
            elif self.goal_status != None:
                return 'aborted'

    def found_balls(self):
        self.ball_status = True
        self.action_client.cancel_goal()

        self._trigger_cond.acquire()
        self._trigger_cond.notify()
        self._trigger_cond.release()

    def done_cb(self, term_state, result):
        self.goal_status = term_state

        self._trigger_cond.acquire()
        self._trigger_cond.notify()
        self._trigger_cond.release()


class TravelState(Concurrence):
    def __init__(self):
        Concurrence.__init__(self,
                input_keys=['target_pose'],
                outcomes=['succeeded', 'preempted', 'aborted'],
                default_outcome='aborted',
                child_termination_cb = self.child_termination_cb,
                outcome_cb = self.outcome_cb
                )

        self.sm_move = StateMachine(
                input_keys=['target_pose'],
                outcomes=['succeeded', 'preempted', 'aborted']
                )

        self.action_client = SimpleActionClient('fibonacci', FibonacciAction)

        self.move_state = Move(self.action_client, 0.2)
        self.chaser_state = BallChaser(self.action_client, 0.2)
        self.userdata.msg_in = None

        self.watcher = BallTracker(self.move_state, self.chaser_state)

        with self.sm_move:
            StateMachine.add('MOVE_TO_GOAL', self.move_state,
                    transitions={'balls_found':'CHASE_BALLS'}
                    )
            StateMachine.add('CHASE_BALLS', self.chaser_state,
                    transitions={
                        'no_more_balls':'MOVE_TO_GOAL',
                        'succeeded':'CHASE_BALLS'
                        }
                    )

        with self:
            Concurrence.add('SM_MOVE', self.sm_move)
            Concurrence.add('BALL_WATCHER', self.watcher)

    def execute(self):
        self.action_client.wait_for_server()

        return Concurrence.execute(self)

    def child_termination_cb(self, outmap):
        if outmap['SM_MOVE']!=None:
            self.watcher.request_preempt()
            return True
        else:
            rospy.logwarn('Unexpected termination')
            return False

    def outcome_cb(self, outmap):
        return outmap['SM_MOVE']








def main():
    rospy.init_node('traveltest')
    sm_travel = TravelState()
    sm_travel.userdata.target_pose = 10
    sm_travel.execute()

if __name__=='__main__':
    main()
