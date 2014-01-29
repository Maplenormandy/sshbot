#!/usr/bin/env python
import roslib; roslib.load_manifest('paralympics')
import rospy
from geometry_msgs.msg import Twist, Pose, PointStamped, PoseStamped
from std_msgs.msg import Header, Empty
from profit.msg import *
from move_base_msgs.msg import *
from b2b.msg import IRStamped, IRArray
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


class BallWatcher(SensorState):
    def __init__(self, move_state):
        SensorState.__init__(self, '/profit/balls_raw', BallArray, 0.2,
                output_keys=['ball_pose'],
                outcomes=['found_balls', 'preempted']
                )

        self.move_state = move_state
        self.balls = None
        self.ball_header = None

        self.failed_balls = 0

        self.seq = 0

    def loop(self, msg, ud):
        if self.failed_balls>0:
            self.seq += 1
        else:
            self.seq = 0

        if self.failed_balls < 1:
            if len(msg.balls)>0:
                return 'found_balls'

        if self.seq % 80 == 0:
            self.failed_balls -= 1
            if self.failed_balls < 0:
                self.failed_balls = 0

    def execute(self, ud):
        self.seq = 0
        return SensorState.execute(self, ud)


class ChaseBalls(SensorState):
    def __init__(self, cmd_vel_pub, watcher):
        SensorState.__init__(self, '/profit/balls_raw', BallArray, 0.2)
        self._cmd_vel = cmd_vel_pub

        self.irlock = threading.RLock()
        self.irmsg = None

        self.watcher = watcher
        rospy.Subscriber('/overspeed', Empty, self.overspeeded)
        self.overspeed = False
        
    def overspeeded(self, msg):
        self.overspeed = True

    def updateIRs(self, msg):
        self.irlock.acquire()
        self.irmsg = msg
        self.irlock.release()

    def execute(self, ud):
        self.overspeed = False
        self.xl = 0.0
        self.rl = 0.0
        self.lfl = 0.0
        self.lfdl = 0.0
        self.lostframes = 0
        self.vel = Twist()
        self.chaseframes = 0

        return SensorState.execute(self, ud)

    def loop(self, msg, ud):
        walled = False

        self.irlock.acquire()
        irmsg = self.irmsg
        self.irlock.release()

        vel = Twist()
        self.chaseframes += 1
        
        if self.overspeed:
            self.watcher.failed_balls += 1
            vel.linear.x = -0.1;
            self._cmd_vel.publish(vel)
            rospy.sleep(1.0)
            vel.linear.x = 0.0
            self._cmd_vel.publish(vel)
            rospy.sleep(1.0)
            return 'succeeded'

        if self.chaseframes > 100:
            self.watcher.failed_balls += 1
            return 'succeeded'

        if irmsg != None:
            if irmsg.fwd_l < 0.1:
                vel.angular.z = -0.2
                walled=True
            elif irmsg.fwd_r < 0.1:
                vel.angular.z = 0.2
                walled=True
            elif irmsg.l.fwd < 0.20:
                lf = np.clip(irmsg.l.fwd, 0.05, 0.65) - 0.16
                lfd = (lf-self.lfl) * 16.0

                vel.linear.x = 0.2 + lf
                vel.angular.z = np.clip(lf+lfd*0.25, -0.6, 0.6)

                self.lfl = lf
                self.lfdl = lfd
                walled=True
            elif irmsg.r.fwd < 0.20:
                lf = np.clip(irmsg.r.fwd, 0.05, 0.65) - 0.16
                lfd = (lf-self.lfl) * 16.0

                vel.linear.x = 0.2 + lf
                vel.angular.z = np.clip(lf+lfd*0.25, -0.6, 0.6)

                self.lfl = lf
                self.lfdl = lfd
                walled=True


        if not walled:
            self.lfl = 0.0
            self.lfdl = 0.0
            if len(msg.balls)==0:
                self.lostframes += 1
                if self.lostframes > 16:
                    vel = Twist()
                    self._cmd_vel.publish(vel)
                    self.watcher.failed_balls += 1
                    return 'succeeded'
                else:
                    self.vel.linear.x *= 0.9
                    self.vel.angular.z *= 0.3
            else:
                self.lostframes = 0
                ball = max(msg.balls,
                        key=lambda b: b.point.y-abs(b.point.x)*0.25)


                self.vel.linear.x = self.vel.linear.x*0.5 + 0.1
                self.vel.angular.z = np.clip(
                        self.vel.angular.z*0.5 + ball.point.y*2.0,
                        -0.8, 0.8)

            self._cmd_vel.publish(self.vel)
        else:
            rospy.loginfo('walled')
            self.lostframes += 0.5
            if self.lostframes > 12:
                self.watcher.failed_balls += 1
                vel = Twist()
                self._cmd_vel.publish(vel)
                return 'succeeded'
            self._cmd_vel.publish(vel)

class Move(State):
    def __init__(self, action_client, timeout):
        State.__init__(self,
                input_keys=['target_pose'],
                outcomes=['succeeded', 'preempted', 'aborted']
                )

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

            if self.ball_status:
                return 'balls_found'
            elif self.goal_status == GoalStatus.SUCCEEDED:
                return 'succeeded'
            elif self.goal_status == GoalStatus.PREEMPTED or self.preempt_requested():
                self.action_client.cancel_goal()
                self.service_preempt()
                return 'preempted'
            elif self.goal_status != None:
                self.action_client.cancel_goal()
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
        rospy.loginfo('cancel')
        rospy.loginfo(self.goal)

    def send_goal(self, goal, done_cb):
        rospy.loginfo('start')
        self.goal = goal
        rospy.loginfo(self.goal)
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
                input_keys=['target_pose', 'msg_ball_out'],
                output_keys=['msg_ball_in'],
                outcomes=['succeeded', 'found_balls', 'preempted', 'aborted'],
                default_outcome='aborted',
                child_termination_cb = self.child_termination_cb,
                outcome_cb = self.outcome_cb
                )

        self._cmd_vel = rospy.Publisher('/cmd_vel', Twist)

        self.userdata.msg_ball_out = None

        self.action_client = SimpleActionClient('move_base', MoveBaseAction)
        #self.action_client = PretendActionClient()

        self.move_state = Move(self.action_client, 0.05)
        self.userdata.msg_in = None

        self.watcher = BallWatcher(self.move_state)
        self.chaser_state = ChaseBalls(self._cmd_vel, self.watcher)
        self._ir_pub = rospy.Subscriber('/ir_raw', IRStamped,
                self.chaser_state.updateIRs)

        with self.sm_move:
            Concurrence.add('BALL_WATCHER', self.watcher,
                    remapping={
                        'msg_in':'msg_ball_out',
                        'msg_out':'msg_ball_in'
                        }
                    )
            Concurrence.add('MOVE_TO_GOAL', self.move_state)

        with self:
            StateMachine.add('SM_MOVE', self.sm_move,
                    transitions={'found_balls':'CHASE_BALLS'}
                    )
            StateMachine.add('CHASE_BALLS', self.chaser_state,
                    transitions={'succeeded':'SM_MOVE'},
                    remapping={'msg_in':'msg_ball_in'}
                    )

    def execute(self, ud=None):
        if ud==None:
            ud = self.userdata
        self.action_client.wait_for_server()

        return StateMachine.execute(self, ud)

    def child_termination_cb(self, outmap):
        return True

    def outcome_cb(self, outmap):
        if outmap['BALL_WATCHER'] == 'found_balls':
            return 'found_balls'
        else:
            return outmap['MOVE_TO_GOAL']




def main():
    rospy.init_node('traveltest')
    sm_travel = TravelState()
    target_pose = PoseStamped()
    target_pose.header.stamp = rospy.Time.now()
    target_pose.header.frame_id = "map"
    target_pose.pose.position.x = -0.5
    target_pose.pose.orientation.w = 1.0
    sm_travel.userdata.target_pose = target_pose
    sm_travel.execute()

    """
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist)

    sm_root = StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])
    sm_root.userdata.msg_out = None

    chaser_state = ChaseBalls(cmd_vel_pub)

    ir_pub = rospy.Subscriber('/ir_raw', IRStamped,
            chaser_state.updateIRs)

    with sm_root:
        StateMachine.add('CHASE_BALLS', chaser_state,
                remapping={'msg_in':'msg_out',
                    'msg_out':'msg_in'})


    sm_root.execute()
    """

if __name__=='__main__':
    main()
