#!/usr/bin/env python
import roslib; roslib.load_manifest('paralympics')
import rospy
from geometry_msgs.msg import Twist, PointStamped, Pose, PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import Int16, Empty, Float32
import math
import numpy as np
from sensor_state import SensorState
from profit.msg import *
from smach import *
from smach_ros import *
from b2b.srv import *
from travel_states import *
from nav.srv import *
from square import DriveStraight

__all__ = ['SiloState', 'ReactorState', 'EnemyWallState']

attemptAlign = True

class AlignToReactor(SensorState):
    def __init__(self, cmd_vel_pub):
        SensorState.__init__(self, '/profit/reactor_wall_raw', Wall, 0.1,
                outcomes=['succeeded', 'preempted', 'aborted']
                )

        self._cmd_vel = cmd_vel_pub
        rospy.Subscriber('/overspeed', Empty, self.overspeeded)

    def loop(self, msg, ud):
        avgX = (msg.a.x + msg.b.x + msg.c.x + msg.d.x)/4.0
        diffY = (msg.a.y - msg.d.y) - (msg.b.y - msg.c.y)
        height = (msg.d.y - msg.a.y) + (msg.c.y - msg.b.y)

        """
        rospy.loginfo('---')
        rospy.loginfo(avgX)
        rospy.loginfo(diffY)
        rospy.loginfo(height)
        """

        vel = Twist()

        if self.centering:
            rospy.loginfo('centering')
            if abs(diffY)<0.04:
                vel.angular.z = 0.0
                self.centering = False
            else:
                vel.angular.z = np.clip(-diffY*10.0, -0.6, 0.6)
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
                if abs(avgX+self.avgX0*0.8)<0.04:
                    self.driveCentering = False
                else:
                    vel.angular.z = np.clip(-self.avgX0*2.0, -0.4, 0.4)
            else:
                vel.angular.z = np.clip(-avgX*4.0, -0.6, 0.6)
                rospy.loginfo('driving')
                vel.linear.x = 0.15


        self._cmd_vel.publish(vel)


    def overspeeded(self, msg):
        self.overspeed = True


    def execute(self, ud):
        global attemptAlign
        if not attemptAlign:
            return 'succeeded'
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
                input_keys=['high_balls', 'high_balls_2',
                    'reactor_back_dist', 'reactor_back_speed'],
                outcomes=['succeeded', 'preempted', 'aborted']
                )

        self._cmd_vel = rospy.Publisher('/cmd_vel', Twist)

        self.sm_high = AlignAndQueue(self._cmd_vel)
        self.sm_low = AlignAndQueue(self._cmd_vel)
        self.userdata.msg_in = None

        with self:
            StateMachine.add('SM_HIGH', self.sm_high,
                    remapping={
                        'balls':'high_balls',
                        },
                    transitions={'succeeded':'DUMP_HIGH'}
                    )
            StateMachine.add('DUMP_HIGH', DumpGreenBalls(),
                    transitions={'succeeded':'QUEUE_HIGH_EXTRA'}
                    )
            StateMachine.add('QUEUE_HIGH_EXTRA', QueueGreenBalls(),
                    remapping={'requested':'high_balls_2'},
                    transitions={'succeeded':'DUMP_HIGH_2'}
                    )
            StateMachine.add('DUMP_HIGH_2', DumpGreenBalls(),
                    transitions={'succeeded':'DRIVE_BACK'}
                    )
            StateMachine.add('DRIVE_BACK',
                    DriveStraight(self._cmd_vel),
                    transitions={'succeeded':'succeeded'},
                    remapping={
                        'goal_dist':'reactor_back_dist',
                        'goal_speed':'reactor_back_speed'
                        },
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
        SensorState.__init__(self, '/profit/silo_wall_raw', Wall, 0.1,
                outcomes=['succeeded', 'preempted', 'aborted'])

        self._cmd_vel = cmd_vel_pub

    def loop(self, msg, ud):
        """
        SILO
        """
        avgX = (msg.a.x + msg.b.x + msg.c.x + msg.d.x)/4.0
        diffY = (msg.a.y - msg.d.y) - (msg.b.y - msg.c.y)
        height = (msg.d.y - msg.a.y) + (msg.c.y - msg.b.y)

        rospy.loginfo('---')
        rospy.loginfo(avgX)
        rospy.loginfo(diffY)
        rospy.loginfo(height)

        vel = Twist()

        self.heights = np.roll(self.heights, 1)
        self.heights[0] = height

        if self.centering:
            rospy.loginfo('centering')
            if abs(diffY)<0.04:
                vel.angular.z = 0.0
                self.centering = False
            else:
                vel.angular.z = np.clip(-diffY*10.0, -0.6, 0.6)
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
                if abs(avgX+self.avgX0*0.8)<0.04:
                    self.driveCentering = False
                else:
                    vel.angular.z = np.clip(-self.avgX0*2.0, -0.4, 0.4)
            elif abs(.10-height)<0.05:
                vel = Twist()
                self._cmd_vel.publish(vel)
                return 'succeeded'
            else:
                vel.angular.z = np.clip(-avgX*3.0, -0.4, 0.4)
                rospy.loginfo('driving')
                vel.linear.x = np.clip((height-0.10)*0.6,-0.10,0.10)


        """
        SILO
        """

        self._cmd_vel.publish(vel)

    def execute(self, ud):
        self.heights = np.array([.10]*5)
        global attemptAlign
        if not attemptAlign:
            return 'succeeded'

        """
        SILO
        """

        self.driveCentering = True
        self.driving = False
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

        """
        SILO
        """

class GrabSiloBalls(State):
    def __init__(self, sas_pub):
        State.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'])
        self._sas_pub = sas_pub

    def execute(self, ud):
        msg = Float32()
        msg.data = -0.4
        self._sas_pub.publish(msg)
        rospy.sleep(1.2)
        msg.data = 0.4
        self._sas_pub.publish(msg)
        rospy.sleep(0.9)
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
        rospy.sleep(1.0)
        if ballsColl < 2:
            return 'valid'
        else:
            return 'invalid'

class SiloState(StateMachine):
    def __init__(self):
        StateMachine.__init__(self,
                outcomes=['succeeded', 'preempted', 'aborted'])

        sas_pub = rospy.Publisher('/sas_cmd', Float32)
        cmd_vel = rospy.Publisher('/cmd_vel', Twist)

        self.userdata.msg_in = None

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




class AlignToEnemyWall(SensorState):
    def __init__(self, cmd_vel_pub):
        SensorState.__init__(self, '/profit/enemy_wall_raw', Wall, 0.1,
                outcomes=['succeeded', 'preempted', 'aborted']
                )

        self._cmd_vel = cmd_vel_pub
        rospy.Subscriber('/overspeed', Empty, self.overspeeded)
        self.overspeed = False

    def loop(self, msg, ud):
        avgX = (msg.a.x + msg.b.x + msg.c.x + msg.d.x)/4.0
        diffY = (msg.a.y - msg.d.y) - (msg.b.y - msg.c.y)
        height = (msg.d.y - msg.a.y) + (msg.c.y - msg.b.y)

        """
        rospy.loginfo('---')
        rospy.loginfo(avgX)
        rospy.loginfo(diffY)
        rospy.loginfo(height)
        """

        vel = Twist()

        vel.angular.z = np.clip(-diffY*10.0, -0.6, 0.6)
        rospy.loginfo('driving')
        vel.linear.x = 0.15

        self._cmd_vel.publish(vel)


    def overspeeded(self, msg):
        self.overspeed = True


    def execute(self, ud):
        global attemptAlign
        if not attemptAlign:
            return 'succeeded'

        sub = rospy.Subscriber(self._topic, self._msg_type, self._msg_cb)

        msg = ud.msg_in

        while True:
            if self.preempt_requested():
                self.service_preempt()
                sub.unregister()
                ud.msg_out = msg
                return 'preempted'
            self.overspeed = False

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

class DumpRedBalls(State):
    def __init__(self):
        State.__init__(self,
                output_keys=['dumped'],
                outcomes=['succeeded', 'preempted', 'aborted']
                )

        self.ball_dump = rospy.ServiceProxy('ball_dump', BallDump)

    def execute(self, ud):
        ud.dumped = self.ball_dump('r').dumped
        return 'succeeded'

class EnemyWallState(StateMachine):
    def __init__(self):
        StateMachine.__init__(self,
                outcomes=['succeeded', 'preempted', 'aborted'])

        self._cmd_vel = rospy.Publisher('/cmd_vel', Twist)
        self.userdata.msg_in = None

        with self:
            StateMachine.add('ALIGN_ENEMY', AlignToEnemyWall(self._cmd_vel),
                    transitions={'succeeded':'DUMP_REDS'})
            StateMachine.add('DUMP_REDS', DumpRedBalls(),
                    transitions={'succeeded':'succeeded'}
                    )



def main():
    rospy.init_node('docktest')

    sm_dock = EnemyWallState()
    sm_dock.execute()


if __name__=='__main__':
    main()
