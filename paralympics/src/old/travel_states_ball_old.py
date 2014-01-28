#!/usr/bin/env python
import roslib; roslib.load_manifest('paralympics')
import rospy
from geometry_msgs.msg import Twist, Pose, PointStamped, PoseStamped
from std_msgs.msg import Header
from profit.msg import *
from move_base_msgs.msg import *
import threading
import random
import math
import numpy as np
from sensor_state import SensorState
from profit.msg import BallArray
from smach import *
from smach_ros import *
from actionlib.simple_action_client import SimpleActionClient, GoalStatus
import tf

__all__ = ['TravelState']


class BallTracker(SensorState):
    def __init__(self):
        SensorState.__init__(self, '/profit/balls', BallArray, 0.2,
                output_keys=['ball_pose'],
                outcomes=['found_balls', 'preempted']
                )

        self.balls = None
        self.ball_header = None

        self.tf_listener = tf.TransformListener()
        self.failed_balls = 0

        self.seq = 0

    def loop(self, msg, ud):
        self.seq += 1

        if self.failed_balls < 1:
            if len(msg.balls>0):
                self.balls = list(map(lambda b: b.point, msg.balls))

                ball = min(self.balls, key=lambda b: math.hypot(b.x,b.y))

                ball_pose = PoseStamped()
                ball_pose.header = msg.header
                ball_pose.pose.position = ball
                ball_pose.pose.orientation.w = 1.0

                return 'found_balls'

        if self.seq % 32 == 0:
            self.failed_balls -= 1
            if self.failed_balls < 0:
                self.failed_balls = 0

    def execute(self, ud):
        self.failed_balls = 0
        return SensorState.execute(self, ud)


class Move(State):
    def __init__(self, action_client, timeout, balltracker):
        State.__init__(self,
                input_keys=['target_pose'],
                outcomes=['succeeded', 'preempted', 'aborted']
                )


        self._balltracker = balltracker

        self.action_client = action_client

        self._timeout = timeout
        self._trigger_cond = threading.Condition()

    def execute(self, ud):
        self.goal_status = None
        self.goal = MoveBaseGoal()
        self.goal.target_pose = ud.target_pose

        self.action_client.cancel_goal()
        self.action_client.send_goal(self.goal, self.done_cb)

        self.ball_status = False

        while True:
            self._trigger_cond.acquire()
            self._trigger_cond.wait(self._timeout)
            self._trigger_cond.release()

            if self.goal_status == GoalStatus.SUCCEEDED:
                return 'succeeded'
            elif self.goal_status == GoalStatus.PREEMPTED or self.preempt_requested():
                self._balltracker.failed_balls += 1
                self.action_client.cancel_goal()
                self.service_preempt()
                return 'preempted'
            elif self.goal_status != None:
                self._balltracker.failed_balls += 1
                return 'aborted'

    def done_cb(self, term_state, result):
        self.goal_status = term_state

        self._trigger_cond.acquire()
        self._trigger_cond.notify()
        self._trigger_cond.release()


class PretendActionClient():
    def __init__(self):
        self.pub = rospy.Publisher('/ballgoal', PoseStamped)
        self.goal = None

    def cancel_goal(self):
        rospy.loginfo(self.goal)

    def send_goal(self, goal, done_cb):
        self.goal = goal
        self.pub.publish(goal.target_pose)

    def wait_for_server(self):
        pass


class TravelState(StateMachine):
    def __init__(self):
        StateMachine.__init__(self,
                input_keys=['target_pose'],
                outcomes=['succeeded', 'preempted', 'aborted']
                )

        self.sm_move = Concurrence(
                input_keys=['target_pose'],
                output_keys=['ball_pose'],
                outcomes=['succeeded', 'preempted', 'aborted'],
                default_outcome='aborted',
                child_termination_cb = self.child_termination_cb,
                outcome_cb = self.outcome_cb
                )

        self.action_client = SimpleActionClient('move_base', MoveBaseAction)
        #self.action_client = PretendActionClient()

        self.move_state = Move(self.action_client, 0.2)
        self.chaser_state = BallChaser(self.action_client, 0.2)
        self.userdata.msg_in = None

        self.watcher = BallTracker(self.move_state, self.chaser_state)

        with self.sm_move:
            Concurrence.add('BALL_WATCHER', self.watcher)
            Concurrence.add('MOVE_TO_GOAL', self.move_state)

        with self:
            StateMachine.add('SM_MOVE', self.sm_move)
            StateMachine.add('CHASE_BALLS', self.move_state,
                    transitions={
                        'succeeded':'SM_MOVE',
                        'aborted':'SM_MOVE',
                        'preempted':'SM_MOVE'
                        },
                    remapping={'target_pose':'ball_pose'}
                    )

    def execute(self, ud=None):
        if ud==None:
            ud = self.userdata
        self.action_client.wait_for_server()

        return Concurrence.execute(self, ud)

    def child_termination_cb(self, outmap):
        if outmap['CHASE_BALLS']!=:
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
    target_pose = PoseStamped()
    target_pose.header.stamp = rospy.Time.now()
    target_pose.header.frame_id = "base_link"
    target_pose.pose.position.x = -0.5
    target_pose.pose.orientation.z = 1.0
    sm_travel.userdata.target_pose = target_pose
    sm_travel.execute()

if __name__=='__main__':
    main()
