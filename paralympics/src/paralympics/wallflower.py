#!/usr/bin/env python
import roslib; roslib.load_manifest('paralympics')
import rospy
from geometry_msgs.msg import Twist, TwistStamped
from paralympics.msg import IRStamped
from sensor_msgs.msg import LaserScan
import math
import numpy as np
from sensor_state import SensorState
from smach import *

cfg = None


class FindWall(SensorState):
    def __init__(self, cmd_vel_pub):
        SensorState.__init__(self, '/ir_raw', IRStamped, 0.1,
                outcomes=['found_wall', 'preempted', 'aborted'])

        self._cmd_vel = cmd_vel_pub


    def loop(self, msg, ud):
        if msg.l.mid < 0.10:
            return 'found_wall'

        if msg.fwd < 0.10:
            return 'found_wall'

        vel = Twist()

        if msg.l.mid < 0.20:
            vel.linear.x = np.clip(0.20-msg.l.mid, 0.00, 0.06)
            vel.angular.z = np.clip(0.4-msg.l.mid, 0.05, 0.4)
        else:
            vel.linear.x = 0.03
            vel.angular.z = -0.4

        self._cmd_vel.publish(vel)

class Cruise(SensorState):
    def __init__(self, cmd_vel_pub):
        SensorState.__init__(self, '/ir_raw', IRStamped, 0.1,
                outcomes=['lost_wall', 'preempted', 'aborted'])

        self._cmd_vel = cmd_vel_pub

    def execute(self, ud):
        self.lfl = 0.0
        self.lfi = 0.0
        self.lfdl = 0.0
        return SensorState.execute(self, ud)

    def loop(self, msg, ud):

        vel = Twist()

        if msg.fwd < 0.10:
            vel.angular.z = -0.2
        elif msg.l.fwd > 0.29:
            return 'lost_wall'
        else:
            lf = np.clip(msg.l.fwd, 0.05, 0.65) - 0.16
            lf = 0.2*lf + 0.8*self.lfl
            self.lfl = lf
            # TODO better integration and diff
            self.lfi += lf * 0.045
            lfd = (lf-self.lfl) / 0.045
            lfd = lfd*0.2 + self.lfdl*0.8
            self.lfdl = lfd

            if lf < 0:
                vel.linear.x = 0.05 + lf/2.0
            else:
                vel.linear.x = np.clip(0.05 - lf/2.0, 0, 0.05)

            cmd = lf*1.0 + self.lfi*0.0 + lfd*0.25

            vel.angular.z = np.clip(cmd, -0.2, 0.2)

        self._cmd_vel.publish(vel)


class Wallflower(StateMachine):
    def __init__(self, cmd_vel_pub):
        StateMachine.__init__(self,
                outcomes=['preempted', 'aborted'])

        self.userdata.msg_out = None

        with self:
            StateMachine.add('CRUISE', Cruise(cmd_vel_pub),
                    transitions = {'lost_wall':'FIND_WALL'},
                    remapping = {'msg_in':'msg_out',
                                 'msg_out':'msg_in'})
            StateMachine.add('FIND_WALL', FindWall(cmd_vel_pub),
                    transitions = {'found_wall':'CRUISE'},
                    remapping = {'msg_in':'msg_out',
                                 'msg_out':'msg_in'})


if __name__=='__main__':
    rospy.init_node('wallflower')

    cmd_vel = rospy.Publisher("/cmd_vel", Twist)

    sm_wall = Wallflower(cmd_vel)

    sm_wall.execute()

