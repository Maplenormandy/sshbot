#!/usr/bin/env python
import roslib; roslib.load_manifest('paralympics')
import rospy
from geometry_msgs.msg import Twist, PointStamped, Pose, PoseStamped
from std_msgs.msg import Int16, Empty, Float32
from actionlib_tutorials.msg import FibonacciAction
import math
import numpy as np
from sensor_state import SensorState
from profit.msg import *
from smach import *
from smach_ros import *
from b2b.srv import *
from travel_states import *
from nav.srv import *

__all__ = ['DispenserState']


class AlignToReactor(SensorState):
    def __init__(self, cmd_vel_pub):
        SensorState.__init__(self, '/profit/reactor_wall_raw', Wall, 0.1,
                outcomes=['succeeded', 'preempted', 'aborted']
                )

        self._cmd_vel = cmd_vel_pub
        rospy.Subscriber('/overspeed', Empty, self.overspeeded)

    def overspeeded(self, msg):
        self.overspeed = True

    def loop(self, msg, ud):
        rospy.loginfo('---')
        avgX = (msg.a.x + msg.b.x + msg.c.x + msg.d.x)/4.0
        rospy.loginfo(avgX)
        diffY = (msg.a.y - msg.d.y) - (msg.b.y - msg.c.y)
        rospy.loginfo(diffY)
        height = (msg.d.y - msg.a.y) + (msg.c.y - msg.b.y)
        rospy.loginfo(height)

        vel = Twist()
        """
        self.avgXi += avgX / 30.0
        self.avgXi = np.clip(self.avgXi, -0.05, 0.05)


        vel.linear.x = np.clip((0.3-height)*0.4, -0.05, 0.05)
        vel.angular.z = np.clip(-avgX*5.0-diffY*5.0
                -self.avgXi*10.0, -0.2, 0.2)
        """

        if self.centering:
            rospy.loginfo('centering')
            if abs(diffY)<0.01:
                vel.angular.z = 0.0
                self.centering = False
            else:
                vel.angular.z = np.clip(-diffY*15.0, -0.4, 0.4)
        elif not self.aligning and abs(avgX)>0.2 and not self.driving:
            self.avgX0 = avgX
            rospy.loginfo('turning away')
            vel.angular.z = np.clip(avgX*1000.0,-0.4,0.4)
        else:
            if not self.driving:
                self.avgX0 = avgX
            self.driving = True
            rospy.loginfo(self.driving)
            if self.driveCentering:
                rospy.loginfo('drive centering')
                if abs(avgX+self.avgX0)<0.05:
                    self.driveCentering = False
                else:
                    vel.angular.z = np.clip(-self.avgX0*2.0, -0.4, 0.4)
            else:
                vel.angular.z = np.clip(-avgX*2.0, -0.8, 0.8)
                rospy.loginfo('driving')
                vel.linear.x = 0.15


        self._cmd_vel.publish(vel)


    def execute(self, ud):
        self.driveCentering = True
        self.driving = False
        self.overspeed = False
        self.avgXi = 0.0
        self.aligning = False
        self.centering = True

        sub = rospy.Subscriber(self._topic, self._msg_type, self._msg_cb)

        msg = ud.msg_in

        while True:
            if self.preempt_requested():
                self.service_preempt()
                sub.unregister()
                ud.msg_out = msg
                return 'preempted'

            if msg == None:
                self._trigger_cond.acquire()
                self._trigger_cond.wait(self._timeout)
                self._trigger_cond.release()

                msg = self._msg

            if self.overspeed:
                for i in range(0):
                    vel = Twist()
                    rospy.loginfo('running')
                    rospy.sleep(0.5)
                    i += 1

                vel = Twist()
                vel.linear.x = 0.0
                self._cmd_vel.publish(vel)
                return 'succeeded'

            if msg != None:
                self.foundWall = True
                ret = self.loop(msg, ud)
                if ret:
                    sub.unregister()
                    ud.msg_out = msg
                    return ret

                self._msg = None
                msg = None
            elif not self.driving:
                rospy.loginfo(self.driving)
                rospy.loginfo('backing up')
                self.foundWall = False
                self.aligning = True
                vel = Twist()
                vel.linear.x = -0.05
                self._cmd_vel.publish(vel)

class QueueGreenBalls(State):
    def __init__(self):
        State.__init__(self,
                input_keys=['requested'],
                output_keys=['queued'],
                outcomes=['succeeded', 'preempted', 'aborted']
                )

        self.green_ball_srv = rospy.ServiceProxy('green_ball_server', GreenBallService)

    def execute(self, ud):
        ud.queued = self.green_ball_srv(ud.requested).queued
        return 'succeeded'

class DumpGreenBalls(State):
    def __init__(self):
        State.__init__(self,
                output_keys=['dumped'],
                outcomes=['succeeded', 'preempted', 'aborted']
                )

        self.ball_dump = rospy.ServiceProxy('ball_dump', BallDump)

    def execute(self, ud):
        ud.dumped = self.ball_dump('g').dumped
        return 'succeeded'

class ReactorState(StateMachine):
    def __init__(self):
        StateMachine.__init__(self,
                input_keys=['high_balls', 'low_balls'],
                outcomes=['succeeded', 'preempted', 'aborted']
                )

        self._cmd_vel = rospy.Publisher('/cmd_vel', Twist)
        self.userdata.high_balls_2 = 1

        self.sm_high = AlignAndQueue(self._cmd_vel)
        self.sm_low = AlignAndQueue(self._cmd_vel)

        with self:
            StateMachine.add('SM_HIGH', self.sm_high,
                    remapping={
                        'balls':'high_balls',
                        },
                    transitions={'succeeded':'DUMP_HIGH'}
                    )
            StateMachine.add('DUMP_HIGH', DumpGreenBalls(),
                    transitions={'succeeded':'succeeded'}
                    )
            StateMachine.add('QUEUE_HIGH_EXTRA', QueueGreenBalls(),
                    remapping={'requested':'high_balls_2'},
                    transitions={'succeeded':'DUMP_HIGH_2'}
                    )
            StateMachine.add('DUMP_HIGH_2', DumpGreenBalls(),
                    transitions={'succeeded':'succeeded'}
                    )
            #StateMachine.add('SM_LOW', self.sm_low,
            #        remapping={
            #            'balls':'low_balls',
            #            'dist':'low_dist',
            #            },
            #        transitions={'succeeded':'DUMP_LOW'}
            #        )
            #StateMachine.add('DUMP_LOW', DumpGreenBalls())

class AlignAndQueue(Concurrence):
    def __init__(self, cmd_vel_pub):
        Concurrence.__init__(self,
                input_keys=['balls'],
                output_keys=['queued'],
                outcomes=['succeeded', 'preempted', 'aborted'],
                default_outcome='aborted',
                child_termination_cb = self.child_termination_cb,
                outcome_cb = self.outcome_cb
                )
        self._cmd_vel = cmd_vel_pub
        self.userdata.msg_in = None

        with self:
            Concurrence.add('ALIGN', AlignToReactor(self._cmd_vel),
                    )
            Concurrence.add('QUEUE', QueueGreenBalls(),
                    remapping={'requested':'balls'}
                    )


    def child_termination_cb(self, outmap):
        if all(map(lambda x: x=='succeeded', outmap.values())):
            return True
        elif any(map(lambda x: x=='aborted' or x=='preempted', outmap.values())):
            return True
        else:
            return False

    def outcome_cb(self, outmap):
        if all(map(lambda x: x=='succeeded', outmap.values())):
            return 'succeeded'
        elif any(map(lambda x: x=='preempted', outmap.values())):
            return 'preempted'
        else:
            return 'aborted'



class AlignToSilo(SensorState):
    def __init__(self, cmd_vel_pub):
        SensorState.__init__(self, '/profit/silo_wall_raw', PointStamped, 0.1,
                outcomes=['succeeded', 'preempted', 'aborted'])

        self._cmd_vel = cmd_vel_pub

    def execute(self, ud):
        return 'succeeded'

class GrabSiloBalls(State):
    def __init__(self, sas_pub):
        State.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'])
        self._sas_pub = sas_pub

    def execute(self, ud):
        msg = Float32()
        msg.data = 0.5
        self._sas_pub.publish(msg)
        rospy.sleep(0.5)
        msg.data = -0.5
        self._sas_pub.publish(msg)
        rospy.sleep(0.5)
        msg.data = 0.0
        self._sas_pub.publish(msg)

        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'

        return 'succeeded'

    def request_preempt(self):
        msg = Int16()
        msg.data = 0
        self._sas_pub.publish(msg)
        State.request_preempt(self)

""" Eventually, want to make sure that there are balls in the silo
class CheckBallsFromSilo(SensorState):
    def __init__(self):
        SensorState.__init__(self, '/profit/ball_silo_raw',
"""

# TODO remove hack here
ballsColl = 0

class CheckSiloBalls(State):
    def __init__(self):
        State.__init__(self, outcomes=['valid', 'invalid', 'preempted'])

    def execute(self, ud):
        global ballsColl
        ballsColl += 1
        rospy.sleep(2.0)
        if ballsColl < 5:
            return 'valid'
        else:
            return 'invalid'

class DispenserState(StateMachine):
    def __init__(self):
        StateMachine.__init__(self,
                outcomes=['succeeded', 'preempted', 'aborted'])

        sas_pub = rospy.Publisher('/sas_cmd', Float32)
        cmd_vel = rospy.Publisher('/cmd_vel', Twist)

        with self:
            StateMachine.add('ALIGN_SILO', AlignToSilo(cmd_vel),
                    transitions={'succeeded':'CHECK_SILO'})
            StateMachine.add('CHECK_SILO', CheckSiloBalls(),
                    transitions={'valid':'GRAB_SILO',
                        'invalid':'succeeded'})
            StateMachine.add('GRAB_SILO', GrabSiloBalls(sas_pub),
                    transitions={'succeeded':'CHECK_SILO',
                        'aborted':'ALIGN_SILO'}
                    )

@cb_interface(
        output_keys=['silo_pose'],
        outcomes=['succeeded','preempted','aborted']
        )
def getSiloPose(ud):
    posSrv = rospy.ServiceProxy('locator', Locator)
    silo_pose = PoseStamped()
    silo_pose.pose = posSrv().silo
    silo_pose.header.stamp = rospy.Time.now()
    silo_pose.header.frame_id = "map"
    ud.silo_pose = silo_pose
    return 'succeeded'
@cb_interface(
        output_keys=['reactor_pose'],
        outcomes=['succeeded','preempted','aborted']
        )
def getReactorPose(ud):
    posSrv = rospy.ServiceProxy('locator', Locator)
    reactor_pose = PoseStamped()
    reactor_pose.pose = posSrv().reactor1
    reactor_pose.header.stamp = rospy.Time.now()
    reactor_pose.header.frame_id = "map"
    ud.reactor_pose = reactor_pose
    return 'succeeded'

def main():
    rospy.init_node('docktest')

    sm_root = StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])

    with sm_root:
        StateMachine.add('DISPENSER_FIND', CBState(getSiloPose),
                transitions={'succeeded':'DISPENSER_TRAVEL'}
                )
        StateMachine.add('DISPENSER_TRAVEL', TravelState(),
                transitions={'succeeded':'DISPENSER'},
                remapping={'target_pose':'silo_pose'}
                )

        sm_disp = DispenserState()
        StateMachine.add('DISPENSER', sm_disp,
                transitions={'succeeded':'REACTOR1_FIND'}
                )

        StateMachine.add('REACTOR1_FIND', CBState(getReactorPose),
                transitions={'succeeded':'REACTOR1_TRAVEL'}
                )
        StateMachine.add('REACTOR1_TRAVEL', TravelState(),
                transitions={'succeeded':'REACTOR1'},
                remapping={'target_pose':'reactor_pose'}
                )

        sm_reactor = ReactorState()
        sm_root.high_balls = 3
        StateMachine.add('REACTOR1', sm_reactor)

    #sm_disp = DispenserState()
    #sm_disp.execute()
    #sm_reactor = ReactorState()
    #sm_reactor.userdata.high_balls = 3
    #sm_reactor.userdata.low_balls = 1
    #sm_reactor.execute()

    sm_root.execute()

if __name__=='__main__':
    main()
