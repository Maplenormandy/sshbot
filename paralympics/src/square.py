#!/usr/bin/env python
import roslib; roslib.load_manifest('paralympics')
import rospy
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Empty
import math
import numpy as np
from sensor_state import SensorState
from smach import *

__all__ = ['DriveStraight']

class DriveStraight(SensorState):
    def __init__(self, cmd_vel_pub):
        SensorState.__init__(self, '/odom_partial', TwistStamped, 0.1,
                input_keys=['goal_dist', 'goal_speed'])

        self._cmd_vel = cmd_vel_pub

        self._pos0 = None
        self._th0 = None
        self._unit = None
        self.overspeedFrames = 0

    def loop(self, msg, ud):
        if self.overspeed:
            self.overspeedFrames += 1
            if self.overspeedFrames > 50:
                self._cmd_vel.publish(Twist())
                return 'aborted'

        if self._pos0 == None:
            self._th0 = msg.twist.linear.z
            self._pos0 = np.array([msg.twist.angular.x, msg.twist.angular.y])
            self._unit = np.array([math.cos(self._th0), math.sin(self._th0)])

        pos = np.array([msg.twist.angular.x, msg.twist.angular.y])
        s = (pos - self._pos0)

        dist = np.sum(s * self._unit)
        vel = Twist()

        if abs(ud.goal_dist - dist) < 0.005:
            vel.linear.x = 0
            vel.angular.z = 0
            self._cmd_vel.publish(vel)
            return 'succeeded'
        else:
            vel.linear.x = np.clip((ud.goal_dist-dist)*3,
                    -ud.goal_speed, ud.goal_speed)
            vel.angular.z = np.clip((self._th0 - msg.twist.linear.z)/40.0,
                    -ud.goal_speed/2.0, ud.goal_speed/2.0)
            self._cmd_vel.publish(vel)

    def execute(self, ud):
        self.overspeedFrames = 0
        self.overspeed = False
        return SensorState.execute(self, ud)

class TurnInPlace(SensorState):
    def __init__(self, cmd_vel_pub):
        SensorState.__init__(self, '/odom_partial', TwistStamped, 0.1,
                input_keys=['goal_angle', 'goal_spin'])

        self._cmd_vel = cmd_vel_pub

        self._pos0 = None
        self._th0 = None
        self._unit = None

    def loop(self, msg, ud):
        if self._pos0 == None:
            self._th0 = msg.twist.linear.z
            self._pos0 = np.array([msg.twist.angular.x, msg.twist.angular.y])
            self._unit = np.array([math.cos(self._th0), math.sin(self._th0)])

        t = (ud.goal_angle - (msg.twist.linear.z - self._th0)) % (math.pi*2)
        vel = Twist()

        if abs(t) < 0.005:
            vel.linear.x = 0
            vel.angular.z = 0
            self._cmd_vel.publish(vel)
            return 'succeeded'
        else:
            vel.linear.x = 0
            vel.angular.z = np.clip(t, -ud.goal_spin, ud.goal_spin)
            self._cmd_vel.publish(vel)






"""
Test Code
"""
def main():
    rospy.init_node('boxxy')

    cmd_vel = rospy.Publisher("/cmd_vel", Twist)

    sm_root = StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])

    with sm_root:

        sm_square = Sequence(outcomes=['succeeded', 'preempted', 'aborted'],
                connector_outcome = 'succeeded')
        sm_square.userdata.msg_out = None
        sm_square.userdata.goal_dist = 0.3048
        sm_square.userdata.goal_speed = 0.3
        sm_square.userdata.goal_angle = math.pi/2
        sm_square.userdata.goal_spin = 0.4

        with sm_square:
            Sequence.add('STRAIGHT0', DriveStraight(cmd_vel),
                    remapping = {'msg_in':'msg_out',
                                 'msg_out':'msg_in'})

            for i in range(1,4):
                Sequence.add('TURN' + str(i), TurnInPlace(cmd_vel),
                        remapping = {'msg_in':'msg_out',
                                     'msg_out':'msg_in'})
                Sequence.add('STRAIGHT' + str(i), DriveStraight(cmd_vel),
                        remapping = {'msg_in':'msg_out',
                                     'msg_out':'msg_in'})

        StateMachine.add('SQUARE', sm_square)

    outcome = sm_root.execute()

    rospy.spin()

if __name__=='__main__':
    main()



