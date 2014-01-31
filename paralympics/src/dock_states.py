#!/usr/bin/env python
import roslib; roslib.load_manifest('paralympics')
import rospy
from geometry_msgs.msg import Twist, PointStamped, Pose, PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import Int16, Empty, Float32, String
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


@cb_interface(
        outcomes=['succeeded','preempted','aborted']
        )
def rollerCmd(ud, pub, power):
    #pub.publish(Float32(power))
    return 'succeeded'


class AlignToReactor(SensorState):
    def __init__(self, cmd_vel_pub):
        SensorState.__init__(self, '/profit/reactor_wall_raw', Wall, 0.1,
                outcomes=['succeeded', 'preempted', 'aborted', 'realign']
                )

        self._cmd_vel = cmd_vel_pub
        rospy.Subscriber('/overspeed', Empty, self.overspeeded)

    def loop(self, msg, ud):
        avgX = (msg.a.x + msg.b.x + msg.c.x + msg.d.x)/4.0
        diffY = (msg.a.y - msg.d.y) - (msg.b.y - msg.c.y)
        height = (msg.d.y - msg.a.y) + (msg.c.y - msg.b.y)
        avgXd = avgX - self.avgXl
        self.avgXl = avgX
        self.diffYl = diffY

        self.diffYs = np.roll(self.diffYs, 1)
        self.diffYs[0] = diffY

        """
        rospy.loginfo('---')
        rospy.loginfo(avgX)
        rospy.loginfo(diffY)
        rospy.loginfo(height)
        """

        vel = Twist()

        if self.centering:
            rospy.loginfo('centering')
            rospy.loginfo(diffY)
            if abs(diffY)<0.04:
                vel.angular.z = 0.0
                self.centering = False
            else:
                vel.angular.z = np.clip(-diffY*10.0, -0.6, 0.6)
        elif not self.aligning and abs(avgX)>0.2 and not self.driving:
            self.avgX0 = avgX
            rospy.loginfo('turning away')
            vel.angular.z = np.clip(avgX*1000.0,-0.6,0.6)
        else:
            if not self.driving:
                self.avgX0 = avgX
            self.driving = True
            rospy.loginfo(self.driving)
            if self.driveCentering:
                rospy.loginfo('drive centering')
                rospy.loginfo(avgX)
                if (abs(avgX+self.avgX0*0.8)<0.04 or
                        abs(avgX*4.0)+abs(diffY*10.0)<0.1):
                    self.driveCentering = False
                else:
                    vel.angular.z = np.clip(-self.avgX0*2.0, -0.6, 0.6)
            else:
                rospy.loginfo(self.diffYs)
                vel.angular.z = np.clip(-avgX*3.0-avgXd*2.0, -0.6, 0.6)
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
        self.avgX0 = 0.0
        self.aligning = False
        self.centering = True
        self.avgXl = 0.0
        self.diffYl = 0.0
        self.backFrames = 0
        self.diffYs = np.array([0.0]*10)

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
                if all(map(lambda x: x > 0.07, self.diffYs)):
                    return 'realign'
                else:
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
                self.backFrames += 1
                vel = Twist()
                if self.backFrames > 40 or self.overspeed:
                    self._cmd_vel.publish(vel)
                    return 'aborted'
                rospy.loginfo(self.driving)
                rospy.loginfo('backing up')
                self.foundWall = False
                self.aligning = True
                vel.linear.x = -0.15
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
        self._roller_pub = rospy.Publisher('/roller_cmd', Float32)

        self.sm_high = AlignAndQueue(self._cmd_vel)
        self.sm_low = AlignAndQueue(self._cmd_vel)
        self.userdata.msg_in = None

        with self:
            StateMachine.add('SM_HIGH', self.sm_high,
                    remapping={
                        'balls':'high_balls',
                        },
                    #transitions={'succeeded':'DRIVE_BACK'}
                    transitions={
                        'succeeded':'DUMP_HIGH',
                        'realign':'DRIVE_BACK_REALIGN',
                        'aborted':'DRIVE_BACK_ABORTED'
                        }
                    )
            StateMachine.add('DUMP_HIGH', DumpGreenBalls(),
                    transitions={
                        'succeeded':'QUEUE_HIGH_EXTRA'
                        }
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

            StateMachine.add('DRIVE_BACK_REALIGN',
                    DriveStraight(self._cmd_vel),
                    transitions={'succeeded':'SM_HIGH_REALIGN'},
                    remapping={
                        'goal_dist':'reactor_back_dist',
                        'goal_speed':'reactor_back_speed'
                        },
                    )
            StateMachine.add('SM_HIGH_REALIGN', self.sm_high,
                    remapping={
                        'balls':'high_balls',
                        },
                    transitions={
                        'succeeded':'DUMP_HIGH',
                        'realign':'DRIVE_BACK_ABORTED',
                        'aborted':'DRIVE_BACK_ABORTED'
                        }
                    )

            StateMachine.add('DRIVE_BACK_ABORTED',
                    DriveStraight(self._cmd_vel),
                    transitions={'succeeded':'aborted'},
                    remapping={
                        'goal_dist':'reactor_back_dist',
                        'goal_speed':'reactor_back_speed'
                        },
                    )

class AlignAndQueue(Concurrence):
    def __init__(self, cmd_vel_pub):
        Concurrence.__init__(self,
                input_keys=['balls'],
                output_keys=['queued'],
                outcomes=['succeeded', 'preempted', 'aborted', 'realign'],
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
        elif any(map(lambda x: x=='realign', outmap.values())):
            return True
        else:
            return False

    def outcome_cb(self, outmap):
        if all(map(lambda x: x=='succeeded', outmap.values())):
            return 'succeeded'
        elif any(map(lambda x: x=='preempted', outmap.values())):
            return 'preempted'
        elif any(map(lambda x: x=='realign', outmap.values())):
            return 'realign'
        else:
            return 'aborted'


class AlignToSiloBall(SensorState):
    def __init__(self, cmd_vel_pub):
        SensorState.__init__(self, '/profit/balls_raw', BallArray, 0.1,
                outcomes=['succeeded', 'preempted', 'aborted', 'realign'])

        self._cmd_vel = cmd_vel_pub
        self.totalframes = 0

    def loop(self, msg, ud):
        vel = Twist()

        self.totalframes += 1

        if self.totalframes > 80:
            return 'aborted'

        if self.overspeed:
            vel.linear.x = -0.15
            self._cmd_vel.publish(vel)
            rospy.sleep(1.0)
            vel.linear.x = 0.0
            self._cmd_vel.publish(vel)
            return 'realign'
        if len(msg.balls)==0:
            self._cmd_vel.publish(vel)
            return 'aborted'

        ball = min(msg.balls, key=lambda b: abs(b.point.x-2.0*b.point.z))

        rospy.loginfo('---')
        rospy.loginfo(ball.point.z)
        rospy.loginfo(ball.point.y)

        if abs(ball.point.y)<0.07:
            self._cmd_vel.publish(vel)
            return 'succeeded'


        self.radii = np.roll(self.radii, 1)
        self.radii[0] = ball.point.z

        #vel.linear.x = np.clip((.15-ball.point.z)*2.0,-0.08,0.08)
        vel.angular.z = np.clip((ball.point.y)*1.0, -0.6,0.6)

        #self.heights = np.roll(self.heights, 1)
        #self.heights[0] = height


        """
        SILO
        """

        self._cmd_vel.publish(vel)

    def execute(self, ud):
        #vel = Twist()
        #vel.linear.x = -0.06
        #self._cmd_vel.publish(vel)
        #rospy.sleep(0.5)
        #vel.linear.x = 0.0
        #self._cmd_vel.publish(vel)
        self.radii = np.array([.10]*5)
        return SensorState.execute(self, ud)

    def timeout(self, ud):
        self.totalframes += 1
        if self.totalframes > 80:
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
            rospy.loginfo(np.average(self.heights))
            if self.driveCentering:
                rospy.loginfo('drive centering')
                if abs(avgX+self.avgX0*0.8)<0.04:
                    self.driveCentering = False
                else:
                    vel.angular.z = np.clip(-self.avgX0*2.0, -0.6, 0.6)
            elif (height > .13 and height < .20
                    and height < np.average(self.heights)):
                vel = Twist()
                self._cmd_vel.publish(vel)
                return 'succeeded'
            else:
                self.heights = np.roll(self.heights, 1)
                self.heights[0] = height
                vel.angular.z = np.clip(-avgX*2.0, -0.6, 0.6)
                rospy.loginfo('driving')
                vel.linear.x = np.clip(abs(height-0.20)*0.6,0.0,0.10)


        """
        SILO
        """

        self._cmd_vel.publish(vel)

    def execute(self, ud):
        self.heights = np.array([0.0]*10)
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
                vel.linear.x = -0.15
                self._cmd_vel.publish(vel)
            else:
                return 'succeeded'

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
        msg.data = 0.7
        self._sas_pub.publish(msg)
        rospy.sleep(0.7)
        msg.data = 0.5
        self._sas_pub.publish(msg)
        rospy.sleep(2.0)
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

class CheckSiloBalls(SensorState):
    def __init__(self):
        SensorState.__init__(self, '/profit/balls_raw', BallArray, 0.1,
                outcomes=['realign', 'valid', 'invalid', 'preempted'])
        self.lostframes = 0
        self.totalframes = 0
        self.ballsColl = 0

    def loop(self, msg, ud):
        self.totalframes += 1

        if self.totalframes > 120:
            return 'invalid'

        if len(msg.balls) > 0:
            ball = min(msg.balls,
                    key=lambda b: abs(b.point.x-2.0*b.point.z))
            rospy.loginfo(abs(ball.point.x-2.0*ball.point.z))

            if abs(ball.point.x-2.0*ball.point.z) < 0.05:
                rospy.loginfo(abs(ball.point.x-2.0*ball.point.z))
                self.lostframes = 0

                if abs(ball.point.y - np.average(self.center[0]))<0.04:
                    if abs(ball.point.y) < 0.06:
                        return 'valid'
                    else:
                        return 'realign'

                self.center = np.roll(self.center, 1)
                self.center[0] = ball.point.y
            else:
                self.lostframes += 1

        else:
            self.lostframes += 1

        if self.lostframes > 48:
            return 'invalid'


    def execute(self, ud):
        self.ballsColl += 1
        self.totalframes = 0
        self.lostframes = 0
        self.center = np.array([.0]*3)

        if self.ballsColl < 3:
            return SensorState.execute(self, ud)
        else:
            return 'invalid'

class SiloState(StateMachine):
    def __init__(self):
        StateMachine.__init__(self,
                outcomes=['succeeded', 'preempted', 'aborted']
                )

        sas_pub = rospy.Publisher('/sas_cmd', Float32)
        cmd_vel = rospy.Publisher('/cmd_vel', Twist)

        self.userdata.msg_in = None
        self.userdata.silo_back_dist = -.1500
        self.userdata.silo_back_speed = .15

        with self:
            StateMachine.add('ALIGN_SILO', AlignToSilo(cmd_vel),
                    transitions={'succeeded':'ALIGN_SILO_BALL'})
            StateMachine.add('ALIGN_SILO_BALL', AlignToSiloBall(cmd_vel),
                    transitions={
                        'succeeded':'CHECK_SILO',
                        'realign':'ALIGN_SILO',
                        'aborted':'CHECK_SILO'
                        })
            StateMachine.add('CHECK_SILO', CheckSiloBalls(),
                    transitions={
                        'valid':'GRAB_SILO',
                        'realign':'ALIGN_SILO_BALL',
                        'invalid':'DRIVE_BACK'})
            StateMachine.add('GRAB_SILO', GrabSiloBalls(sas_pub),
                    transitions={'succeeded':'CHECK_SILO',
                        'aborted':'ALIGN_SILO'}
                    )
            StateMachine.add('DRIVE_BACK',
                    DriveStraight(cmd_vel),
                    transitions={'succeeded':'succeeded'},
                    remapping={
                        'goal_dist':'silo_back_dist',
                        'goal_speed':'silo_back_speed'
                        },
                    )


# TODO Make it do something earlier
def waitForEnd(msg, ud):
    rospy.sleep(3.0)
    return 'invalid'

class AlignToEnemyWall(SensorState):
    def __init__(self, cmd_vel_pub):
        SensorState.__init__(self, '/profit/enemy_wall_raw', Wall, 0.1,
                outcomes=['succeeded', 'preempted', 'aborted']
                )

        self._cmd_vel = cmd_vel_pub
        self.lostframes = 0

    def loop(self, msg, ud):
        avgX = (msg.a.x + msg.b.x + msg.c.x + msg.d.x)/4.0
        diffY = (msg.a.y - msg.d.y) - (msg.b.y - msg.c.y)
        height = (msg.d.y - msg.a.y) + (msg.c.y - msg.b.y)

        if self.overspeed:
            vel = Twist()
            vel.linear.x = 0.0
            self._cmd_vel.publish(vel)
            return 'succeeded'

        """
        rospy.loginfo('---')
        rospy.loginfo(avgX)
        rospy.loginfo(diffY)
        rospy.loginfo(height)
        """

        vel = Twist()

        vel.angular.z = np.clip(-diffY*6.0, -0.6, 0.6)
        rospy.loginfo('driving')
        vel.linear.x = 0.15

        self._cmd_vel.publish(vel)

    def execute(self, ud):
        self.lostframes = 0
        return SensorState.execute(self, ud)


    def overspeeded(self, msg):
        self.overspeed = True

    def timeout(self, ud):
        self.lostframes += 1
        if self.lostframes > 16:
            return 'aborted'

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
        self._roller_pub = rospy.Publisher('/roller_cmd', Float32)
        self.userdata.msg_in = None

        with self:
            StateMachine.add('ROLLER_OFF',
                    CBState(rollerCmd,
                        cb_args=[self._roller_pub,0.0]
                        ),
                    transitions={'succeeded':'ALIGN_ENEMY'}
                    )
            StateMachine.add('ALIGN_ENEMY',
                    AlignToEnemyWall(self._cmd_vel),
                    transitions={'succeeded':'DUMP_REDS'})
            StateMachine.add('WAIT_FOR_END',
                    SensorState('/game_status', String, 0.1,
                        outcomes=['invalid','preempted']
                        ),
                    transitions={'invalid':'DUMP_REDS'}
                )
            StateMachine.add('DUMP_REDS', DumpRedBalls(),
                    transitions={'succeeded':'succeeded'}
                    )



def main():
    rospy.init_node('docktest')

    #sm_dock = SiloState()
    #sm_dock.execute()

    #sm_dock = ReactorState()
    #sm_dock.userdata.high_balls = 3
    #sm_dock.userdata.high_balls_2 = 1
    #sm_dock.userdata.reactor_back_dist = -.1500
    #sm_dock.userdata.reactor_back_speed = 0.14

    sm_dock = EnemyWallState()

    sm_dock.execute()


if __name__=='__main__':
    main()
