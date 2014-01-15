#!/usr/bin/env python

import roslib; roslib.load_manifest('paralympics')
import rospy
import smach
import smach_ros
from geometry_msgs.msg import Twist

class StartDriveStraightState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted', 'aborted', 'done'], input_keys=['distance', 'bot_speed', 'feedback'], output_keys=['run_time', 'direction'])
        
    def execute(self, userdata):
        rospy.loginfo('Initializing DriveStraight')

        t_bot = abs(userdata.distance)/userdata.bot_speed
        if abs(userdata.distance)<.001:
            rospy.loginfo('yo')
            return 'done'
        userdata.direction = userdata.distance/abs(userdata.distance)      
        userdata.run_time = t_bot
        return 'succeeded'

class DriveStraightState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted', 'aborted', 'working'], input_keys=['run_time', 'direction', 'start_time', 'distance', 'bot_speed', 'feedback', 'start_x', 'start_y', 'Kp'])

    def execute(self, userdata):
        pub = rospy.Publisher("cmd_vel", Twist)
        rospy.loginfo('Driving to distance')
        r = rospy.Rate(30)
        pos = rospy.wait_for_message("odom_partial", TwistStamped)
        vel_set = 0
        if userdata.feedback:
            curr_dist = (((pos.linear.x-userdata.start_x)**2+(pos.linear.y-userdata.start_y)**2)**0.5)
            if (curr_dist > (abs(userdata.distance)-.002)):
                return 'succeeded'
            v_wheel = userdata.Kp*userdata.direction*(abs(userdata.distance)-curr_dist) 
            vel_set = min(abs(userdata.bot_speed), abs(v_wheel))*userdata.direction
            #TODO actually sen cmd_vel msg
            if (rospy.get_time()-userdata.start_time)>10*userdata.run_time:
                return 'aborted'
        else:
            if ((rospy.get_time()-userdata.start_time)>userdata.run_time):
                return 'succeeded'
            vel_set = userdata.direction*userdata.bot_speed
        
        vel_msg = Twist()
        vel_msg.linear.x = vel_set
        pub.publish(vel_msg) 
        
        r.sleep()    
        
        return 'working'

class DoneDriveStraightState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'])

    def execute(self, userdata):
        rospy.loginfo('Done driving straight')
        return 'succeeded'

def main():
    rospy.init_node('driving_straight')
    
    pos = rospy.wait_for_message("odom_partial", TwistStamped)
    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])
    
    #sm.userdata.run_time =0 
    sm.userdata.Kp=1
    sm.userdata.feedback=True
    sm.userdata.bot_speed = 0.2
    sm.userdata.distance = -2
    sm.userdata.start_time = rospy.get_time()
    sm.userdata.start_x = pos.linear.x
    sm.userdata.start_y = pos.linear.y
   
    with sm:
        smach.StateMachine.add('INIT', StartDriveStraightState(), transitions={'succeeded':'STRAIGHT PID', 'preempted':'preempted', 'aborted':'aborted', 'done':'DONE STRAIGHT'}, remapping={'distance':'distance', 'bot_speed':'bot_speed', 'feedback':'feedback', 'direction':'direction', 'run_time':'run_time'})
        smach.StateMachine.add('STRAIGHT PID', DriveStraightState(), transitions={'succeeded':'DONE STRAIGHT', 'preempted':'preempted', 'aborted':'aborted', 'working':'STRAIGHT PID'}, remapping={'start_time':'start_time', 'distance':'distance', 'bot_speed':'bot_speed', 'run_time':'run_time', 'feedback':'feedback', 'start_x':'start_x', 'start_y':'start_y', 'Kp':'Kp', 'direction':'direction'})
        smach.StateMachine.add('DONE STRAIGHT', DoneDriveStraightState(), transitions={'succeeded':'succeeded', 'preempted':'preempted', 'aborted':'aborted'})

    outcome = sm.execute()


if __name__ == '__main__':
    main()
