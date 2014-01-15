#!/usr/bin/env python

import roslib; roslib.load_manifest('paralympics')
import rospy
import smach
import smach_ros

"""
# define state Foo
class Foo(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state FOO')
        if self.counter < 3:
            self.counter += 1
            return 'outcome1'
        else:
            return 'outcome2'


# define state Bar
class Bar(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])

    def execute(self, userdata):
        rospy.loginfo('Executing state BAR')
        return 'outcome1'





def main():
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('FOO', Foo(),
                transitions={'outcome1':'BAR', 'outcome2':'outcome4'})
        smach.StateMachine.add('BAR', Bar(),
                transitions={'outcome1':'FOO'})

        # Execute SMACH plan
    outcome = sm.execute()
"""

class DriveStraight(smach.SPAState):
    def __init__(self):
        smach.SPAState.__init__(self, input_keys=['distance']
                                      output_keys=[])

    def execute(self, userdata):
        r = rospy.Rate(10)
        for i in range(userdata.distance):
            print i
            r.sleep()
        return 'succeeded'

class TurnPoint(smach.SPAState):
    def __init__(self):
        smach.SPAState.__init__(self, input_keys=['distance']
                                      output_keys=[])

    def executive(self, userdata):
        r = rospy.Rate(10)
        for i in range(userdata.distance):
            if self.preempt_requested():
                return 'preempted'
            print i
            r.sleep()
        return 'succeeded'



def main():
    rospy.init_node('paralympics_state_machine')

    # Create SMACH state machine
    sm_root = smach.StateMachine(outcomes=['succeeded, preempted, aborted'])
    sis = smach_ros.IntrospectionServer('paralympics', sm, '/sm_root')
    sis.start()



    outcome = sm.execute()
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
