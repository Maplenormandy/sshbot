#!/usr/bin/env python
import roslib; roslib.load_manifest('paralympics')
import rospy
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import String, UInt16
import math
import numpy as np
from sensor_state import SensorState
from profit.msg import BallArray
from smach import *
from smach_ros import *
from rospy_tutorials.srv import *
import thread
import threading

__all__ = ['InitSystems']

@cb_interface(
        input_keys=['x','y'],
        output_keys=['sum'],
        outcomes=['succeeded', 'preempted', 'aborted']
        )
def sort_reactors(ud):
    rospy.wait_for_service('add_two_ints')
    try:
        add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
        ud.sum = add_two_ints(ud.x, ud.y).sum
        return 'succeeded'
    except rospy.ServiceException, e:
        rospy.logerror("Service call failed: %s"%e)
        return 'aborted'

@cb_interface(
        outcomes=['succeeded', 'aborted'])
def botclient_init(ud):
    rospy.sleep(1.0)
    return 'succeeded'

class InitSystems(Concurrence):
    def __init__(self):
        Concurrence.__init__(self,
                output_keys=['total_dist'],
                outcomes=['succeeded', 'aborted'],
                default_outcome='aborted',
                child_termination_cb = self.child_termination_cb,
                outcome_cb = self.outcome_cb)

        self.userdata.dist1 = 3
        self.userdata.dist2 = 4

        with self:
            Concurrence.add('SORT_REACTORS',
                    CBState(sort_reactors),
    # TODO Threading business
                    remapping={'x':'dist1',
                               'y':'dist2',
                               'sum':'total_dist'}
                    )
            Concurrence.add('BOTCLIENT_INIT',
                    CBState(botclient_init)
                    )

    def child_termination_cb(self, outmap):
        if any(map(lambda x: x=='aborted', outmap.values())):
            return True
        elif all(map(lambda x: x=='succeeded', outmap.values())):
            return True
        else:
            return False

    def outcome_cb(self, outmap):
        if any(map(lambda x: x=='aborted', outmap.values())):
            return 'aborted'
        else:
            return 'succeeded'




def main():
    rospy.init_node('systemtest')
    sm_root = StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])
    sm_root.userdata.msg_in = None

    with sm_root:
        StateMachine.add('BALL_HANDLER', BallHandler())

    sm_root.execute()


if __name__=='__main__':
    main()
