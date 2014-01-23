#!/usr/bin/env python
import roslib; roslib.load_manifest('paralympics')
import rospy
from geometry_msgs.msg import Twist, TwistStamped
import math
import numpy as np
from sensor_state import SensorState
from init_state import InitState
from wallflower import Wallflower
from profit.msg import BallArray
from smach import *
from smach_ros import *

class ChaseBalls(SensorState):
    def __init__(self, cmd_vel_pub):
        SensorState.__init__(self, '/balls', BallArray, 0.2)
        self._cmd_vel = cmd_vel_pub

    def execute(self, ud):
        self.xl = 0.0
        self.rl = 0.0
        self.lostframes = 0
        self.vel = Twist()
        return SensorState.execute(self, ud)

    def loop(self, msg, ud):
        if len(msg.balls)==0:
            self.lostframes += 1
            if self.lostframes > 24:
                return 'succeeded'
            else:
                self.vel.linear.x *= 0.25
                self.vel.angular.z *= 0.3

        else:
            maxScore = 0.0
            maxBall = None
            for ball in msg.balls:
                score = ball.y + abs(ball.x)*0.2 + ball.r
                if score > maxScore:
                    maxScore = score
                    maxBall = ball

            maxBall.x = maxBall.x*0.9 + self.xl*0.1
            maxBall.r = maxBall.r*0.5 + self.rl*0.5

            self.xl = maxBall.x
            self.rl = maxBall.r

            self.vel.linear.x = self.vel.linear.x*0.5 + 0.025
            self.vel.angular.z = self.vel.angular.z*0.5 - maxBall.x*0.25

            self._cmd_vel.publish(self.vel)


class WatchBalls(SensorState):
    def __init__(self):
        SensorState.__init__(self, '/balls', BallArray, 0.2,
                outcomes=['found_ball'])

    def loop(self, msg, ud):
        if len(msg.balls)>0:
           return 'found_ball'


""" Template for nav state:
        SimpleActionState.__init__(self,
                'move_base',
                MoveBaseAction,
                goal_slots=['target_pose'])
"""


def main():
    rospy.init_node('supersonic')

    cmd_vel = rospy.Publisher("/cmd_vel", Twist)
    sm_root = StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])

    with sm_root:

        sm_init = InitState()

        StateMachine.add('INIT',
                sm_init,
                remapping={'total_dist':'sm_dist'}
                )

    sm_root.execute()
    rospy.loginfo(sm_root.userdata.sm_dist)

if __name__=='__main__':
    main()
