#! /usr/bin/env python

import roslib; roslib.load_manifest('paralympics')
import rospy

from geometry_msgs.msg import Twist

import actionlib

import paralympics.msg

class DriveStraightAction(object):
    _feedback = paralympics.msg.DriveStraightFeedback()
    _result = paralympics.msg.DriveStraightResult()

    def __init(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(
                self._action_name,
                paralympics.msg.DriveStraightAction,
                execute_cb=self.execute_cb,
                auto_start = False)
        self._as.start()
        self.x=0
        self.y=0
        self.x_init=0
        self.y_init=0
        self.initFlag=False

        rospy.Subscriber("odom_partial", 
                geometry_msgs.msg.TwistStamped, 
                pose_cb, self)
        self.pub=rospy.Publisher("cmd_vel", Twist)

    def pose_cb(data, arg):
        if not initFlag:
            arg.x_init=data.linear.x
            arg.y_init=data.linear.y
            arg.initFlag=True

        arg.x=data.linear.x
        arg.y=data.linear.y

    def execute_cb(self, goal):
        r = rospy.Rate(30)
        done = False
        success = False

        axleR = 10.75*2.54/200.0
        wheelR = 3.875*2.54/200.0

        # Determine maximum speed, etc...
        if distance<.001:
            direction = 1
        else:
            direction = goal.distance/abs(goal.distance)
        t_bot = goal.distance / goal.bot_speed
        t_max = t_bot
        v_wheel = abs(goal.distance / t_max)
        if v_wheel > goal.wheel_speed:
            t_max = t_max * v_wheel / goal.wheel_speed

        bot_speed = goal.distance / t_max

        while not done:
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                break

            if not rospy.is_shutdown():
                #check if we have recieved an initial pose
                if not self.initFlag:
                    r.sleep()
                    break
 
                #compute cartesian distance traveled
#TODO account for negative distance goals less naively
                self._feedback.distance=direction*pow((self.x-self.x_init)**2+(self.y-self.y_init)**2,0.5)

                motorSet = min(Kp*(goal.distance-self._feedback.distance), wheel_speed)

                #TODO publish to cmd_vel
                motorPub=Twist()
                motorPub.linear=Vector3(motorSet,0,0)
                motorPub.angular=Vector3(0,0,0)
                self.pub(motorPub)
                self._as.publish_feedback(self._feedback)

                r.sleep()

            if success:
                self._result.distance = self._feedback.distance
                rospy.loginfo('%s: Succeeded' % self._action_name)
                self._as.set_succeeded(self._result)

            else:
                #rospy quit, abort
                self._as.set_aborted()
                r.sleep()

if __name__ == '__main__':
    rospy.init_node('drive_straight')
    DriveStraightAction(rospy.get_name())
    rospy.spin()
