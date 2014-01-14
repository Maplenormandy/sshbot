#!/usr/bin/env python

import roslib; roslib.load_manifest('paralympics')
import rospy
import smach
import smach_ros

class StartDriveStraightState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'])

    def execute(self, userdata):
        rospy.loginfo('Initializing DriveStraight')
        return 'succeeded'

class DriveStraightState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted', 'aborted', 'working'])

    def execute(self, userdata):
        rospy.loginfo('Driving to distance')
        return 'succeeded'

class DoneDriveStraightState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'])

    def execute(self, userdata):
        rospy.loginfo('Done driving straight')
        return 'succeeded'

def main():
    rospy.init_node('driving_straight')

    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])
    with sm:
        smach.StateMachine.add('INIT', StartDriveStraightState(), transitions={'succeeded':'STRAIGHT PID', 'preempted':'preempted', 'aborted':'aborted'})
        smach.StateMachine.add('STRAIGHT PID', DriveStraightState(), transitions={'succeeded':'DONE STRAIGHT', 'preempted':'preempted', 'aborted':'aborted', 'working':'STRAIGHT PID'})
        smach.StateMachine.add('DONE STRAIGHT', DoneDriveStraightState(), transitions={'succeeded':'succeeded', 'preempted':'preempted', 'aborted':'aborted'})

    outcome = sm.execute()


if __name__ == '__main__':
    main()
