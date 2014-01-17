#!/usr/bin/env python
import roslib; roslib.load_manifest('paralympics')
import rospy
from geometry_msgs.msg import Twist, TwistStamped
import math
import numpy as np
from sensor_state import SensorState
from wallflower import Wallflower
from std_msgs.msg import Empty
from smach import *

class ChaseBalls(SensorState):
    def __init__(self):
        SensorState.__init__(self, '/balls', Empty, 0.2)

    def loop(self, msg, ud):
        # TODO Chase Balls
        pass

class WatchBalls(SensorState):
    def __init__(self):
        SensorState.__init__(self, '/balls', Empty, 0.2,
                outcomes=['found_ball'])

    def loop(self, msg, ud):
        # TODO Check if any balls were watched
        pass

def main():
    rospy.init_node('mach_one')

    cmd_vel = rospy.Publisher("/cmd_vel", Twist)
    sm_root = StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])

    with sm_root:

        def child_term_cb(outmap):
            return True

        def out_cb(outmap):
            if outmap['WATCH_BALLS'] == 'found_ball':
                return 'found_ball'
            else:
                return outmap['WALLFLOWER']

        sm_wall_cc = Concurrence(outcomes=['found_ball', 'preempted', 'aborted'],
                default_outcome = 'aborted',
                input_keys = ['msg_ball_in'],
                output_keys = ['msg_ball_out'],
                child_termination_cb = child_term_cb,
                outcome_cb = out_cb)

        sm_wall = Wallflower(cmd_vel)

        with sm_wall_cc:
            Concurrence.add('WALLFLOWER', sm_wall)
            Concurrence.add('WATCH_BALLS', WatchBalls(),
                    remapping={'msg_in':'msg_ball_in',
                               'msg_out':'msg_ball_out'})

        sm_root.userdata.msg_ball_out = None

        StateMachine.add('WALLFLOWER_CC', sm_wall_cc,
                transitions={'found_ball':'CHASE_BALLS'},
                remapping={'msg_ball_in':'msg_ball_out',
                           'msg_ball_out':'msg_ball_in'})

        StateMachine.add('CHASE_BALLS', ChaseBalls(),
                transitions={'succeeded':'WALLFLOWER_CC',
                             'aborted':'WALLFLOWER_CC'},
                remapping={'msg_in':'msg_ball_out',
                           'msg_out':'msg_ball_in'})

    sm_root.execute()
    rospy.spin()

if __name__=='__main__':
    main()
