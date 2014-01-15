#! /usr/bin/env python

import roslib; roslib.load_manifest('paralympics')
import rospy

import actionlib

import paralympics.msg

class DriveDistanceAction(object):
    _feedback = paralympics.msg.DriveDistanceFeedback()
    _result = paralympics.msg.DriveDistanceResult()

    def __init(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(
                self._action_name,
                paralympics.msg.DriveDistanceAction,
                execute_cb=self.execute_cb,
                auto_start = False)
        self._as.start()

    def execute_cb(self, goal):
        r = rospy.Rate(30)
        done = False
        success = True

        axleR = 10.75*2.54/200.0
        wheelR = 3.875*2.54/200.0

        # Determine maximum speed, etc...
        t_bot = goal.distance / goal.bot_speed
        t_turn = goal.turn_angle / goal.turn_speed
        t_max = max(abs(t_bot), abs(t_turn))
        v_wheel = abs(goal.distance / t_max)+abs(goal.turn_angle / t_max)*axleR
        if v_wheel > goal.wheel_speed:
            t_max = t_max * v_wheel / goal.wheel_speed

        bot_speed = goal.distance / t_max
        turn_angle = goal.turn_angle / t_max

        while not done:
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                break

            self._as.publish_feedback(self._feedback)

            r.sleep()

        if success:
            self._result.distance = self._feedback.distance
            self._result.turn_angle = self._feedback.turn_angle
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)


if __name__ == '__main__':
    rospy.init_node('drive_distance')
    DriveDistanceAction(rospy.get_name())
    rospy.spin()
