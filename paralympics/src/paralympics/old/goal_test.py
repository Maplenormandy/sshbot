#!/usr/bin/env python
import roslib; roslib.load_manifest('paralympics')
import rospy
import actionlib
from actionlib_tutorials.msg import *

def done_cb(term_state, result):
    rospy.loginfo({'term_state': term_state, 'result': result})

def active_cb():
    rospy.loginfo('active')

def feedback_cb(feedback):
    rospy.loginfo(feedback)

def main():
    rospy.init_node('goal_testing')
    client = actionlib.SimpleActionClient('fibonacci', FibonacciAction)
    client.wait_for_server()


    goal = FibonacciGoal()
    goal.order = 5

    client.send_goal(goal, done_cb, active_cb, feedback_cb)

    rospy.sleep(3.0)

    client.cancel_goal()
    goal2 = FibonacciGoal()
    goal2.order = 3

    rospy.loginfo(client.send_goal_and_wait(goal2))

    rospy.sleep(2.0)

if __name__=='__main__':
    main()

