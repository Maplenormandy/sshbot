#!/usr/bin/env python
import roslib; roslib.load_manifest('paralympics')
import rospy
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import String
from sensor_state import SensorState
from system_states import InitSystems
from travel_states import TravelState
from dock_states import SiloState, ReactorState, EnemyWallState
from smach import *
from smach_ros import *
from nav.srv import *
from geometry_msgs.msg import Twist, PointStamped, Pose, PoseStamped, PoseWithCovarianceStamped


@cb_interface(
        input_keys=['target'],
        output_keys=['target_pose'],
        outcomes=['succeeded','preempted','aborted']
        )
def getTargetPose(ud):
    posSrv = rospy.ServiceProxy('locator', Locator)
    target_pose = PoseStamped()
    rospy.loginfo(ud.target)
    if ud.target == "reactor1":
        target_pose.pose = posSrv().reactor1
    elif ud.target == "reactor2":
        target_pose.pose = posSrv().reactor2
    elif ud.target == "reactor3":
        target_pose.pose = posSrv().reactor3
    elif ud.target == "silo":
        target_pose.pose = posSrv().silo
    elif ud.target == "enemy":
        target_pose.pose = posSrv().opponent
    target_pose.header.stamp = rospy.Time.now()
    target_pose.header.frame_id = "map"
    ud.target_pose = target_pose
    return 'succeeded'

class TheDecider(State):
    def __init__(self):
        State.__init__(self,
                outcomes=[
                    'silo','reactor1','reactor2','reactor3','enemy',
                    'succeeded','preempted','aborted'
                    ]
                )
        # TODO Put back reactors 2 and 3
        self.silo_tries = 0
        self.reactor_tries = [0, -1, -1]
        self.enemy_tries = 0

    def execute(self, ud):
        silo_valid = self.silo_tries >= 0 and self.silo_tries < 2
        reactor_valid = map(lambda x: x>=0 and x<2, self.reactor_tries)

        rospy.loginfo('---')
        rospy.loginfo(self.silo_tries)
        rospy.loginfo(self.reactor_tries)
        rospy.loginfo(self.enemy_tries)

        if (silo_valid and
                self.silo_tries <= sum(map(abs,self.reactor_tries))):
            return 'silo'
        else:
            minTries = min(map(abs,self.reactor_tries))
            for i in range(3):
                if (reactor_valid[i] and
                        self.reactor_tries[i]<=minTries):
                    return "reactor" + str(i+1)

            if self.enemy_tries >= 0 and self.enemy_tries < 2:
                return 'enemy'
            else:
                return 'succeeded'

    def targetAborted(self, ud):
        if ud.target == "reactor1":
            self.reactor_tries[0] += 1
        elif ud.target == "reactor2":
            self.reactor_tries[1] += 1
        elif ud.target == "reactor3":
            self.reactor_tries[2] += 1
        elif ud.target == "silo":
            self.silo_tries += 1
        elif ud.target == "enemy":
            self.enemy_tries += 1
        return 'continue'

    def targetSucceeded(self, ud):
        if ud.target == "reactor1":
            self.reactor_tries[0] = -1
        elif ud.target == "reactor2":
            self.reactor_tries[1] = -1
        elif ud.target == "reactor3":
            self.reactor_tries[2] = -1
        elif ud.target == "silo":
            self.silo_tries = -1
        elif ud.target == "enemy":
            self.enemy_tries = -1
        return 'continue'

class ActionPackage(Sequence):
    def __init__(self, name, meat, input_keys=[]):
        Sequence.__init__(self,
                outcomes=['succeeded','aborted','preempted'],
                connector_outcome='succeeded'
                )

        self.name = name
        self.uname = name.upper()
        self.userdata.target = name

        with self:
            Sequence.add(name.upper() + '_FIND', CBState(getTargetPose))
            Sequence.add(name.upper() + '_TRAVEL', TravelState())

            sm_disp = SiloState()
            Sequence.add(name.upper(), meat)

    def hookRoot(self, sm_root, thinker):
        StateMachine.add('SM_' + self.uname, self,
                transitions={
                    'succeeded':self.uname + '_SUCCEEDED',
                    'preempted':self.uname + '_ABORTED',
                    'aborted':self.uname + '_ABORTED'
                    }
                )
        StateMachine.add(self.uname + '_ABORTED',
                CBState(thinker.targetAborted,
                    input_keys=['target'],
                    outcomes=['continue']),
                transitions={'continue':'THINKER'},
                remapping={'target':self.name}
                )
        StateMachine.add(self.uname + '_SUCCEEDED',
                CBState(thinker.targetSucceeded,
                    input_keys=['target'],
                    outcomes=['continue']),
                transitions={'continue':'THINKER'},
                remapping={'target':self.name}
                )



def waitForStart(msg, ud):
    return 'valid'



def main():
    rospy.init_node("supersonic")

    cmd_vel = rospy.Publisher("/cmd_vel", Twist)
    sm_root = StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])
    sm_root.userdata.silo = "silo"
    sm_root.userdata.reactor1 = "reactor1"
    sm_root.userdata.reactor2 = "reactor2"
    sm_root.userdata.reactor3 = "reactor3"
    sm_root.userdata.enemy = "enemy"

    thinker = TheDecider()


    sm_silo = ActionPackage('silo', SiloState())
    sm_reactor1 = ActionPackage('reactor1', ReactorState())
    sm_reactor2 = ActionPackage('reactor2', ReactorState())
    sm_reactor3 = ActionPackage('reactor3', ReactorState())
    sm_enemy = ActionPackage('enemy', EnemyWallState())

    sm_reactor1.userdata.high_balls = 3
    sm_reactor1.userdata.high_balls_2 = 1
    sm_reactor1.userdata.reactor_back_dist = -0.1334
    sm_reactor1.userdata.reactor_back_speed = 0.15
    sm_reactor2.userdata.high_balls = 3
    sm_reactor2.userdata.high_balls_2 = 1
    sm_reactor2.userdata.reactor_back_dist = -0.1334
    sm_reactor2.userdata.reactor_back_speed = 0.15
    sm_reactor3.userdata.high_balls = 3
    sm_reactor3.userdata.high_balls_2 = 1
    sm_reactor3.userdata.reactor_back_dist = -0.1334
    sm_reactor3.userdata.reactor_back_speed = 0.15

    sm_root.userdata.msg_in = None

    with sm_root:
        StateMachine.add('WAITER',
                SensorState('/start', String, 0.1, loopFn=waitForStart,
                        outcomes=['valid']
                        ),
                transitions={'valid':'THINKER'}
                )
        StateMachine.add('THINKER', thinker,
                transitions={
                    'silo':'SM_SILO',
                    'reactor1':'SM_REACTOR1',
                    'reactor2':'SM_REACTOR2',
                    'reactor3':'SM_REACTOR3',
                    'enemy':'SM_ENEMY'
                    }
                )
        sm_silo.hookRoot(sm_root, thinker)
        sm_reactor1.hookRoot(sm_root, thinker)
        sm_reactor2.hookRoot(sm_root, thinker)
        sm_reactor3.hookRoot(sm_root, thinker)
        sm_enemy.hookRoot(sm_root, thinker)


    sm_root.execute()


if __name__=='__main__':
    main()
