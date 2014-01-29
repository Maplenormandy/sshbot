#!/usr/bin/env python
import roslib; roslib.load_manifest('paralympics')
import rospy
from geometry_msgs.msg import Twist, TwistStamped
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

def main():
    rospy.init_node("supersonic")

    cmd_vel = rospy.Publisher("/cmd_vel", Twist)
    sm_root = StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])
    sm_root.userdata.silo = "silo"
    sm_root.userdata.reactor1 = "reactor1"
    sm_root.userdata.reactor2 = "reactor2"
    sm_root.userdata.reactor3 = "reactor3"
    sm_root.userdata.enemy = "enemy"
    sm_root.userdata.high_balls = 3
    sm_root.userdata.high_balls_2 = 1
    sm_root.userdata.reactor_back_dist = -0.1334
    sm_root.userdata.reactor_back_speed = 0.15

    with sm_root:
        StateMachine.add('SILO_FIND', CBState(getTargetPose),
                transitions={'succeeded':'SILO_TRAVEL'},
                remapping={
                    'target':'silo',
                    'target_pose':'silo_pose'
                    }
                )
        StateMachine.add('SILO_TRAVEL', TravelState(),
                transitions={'succeeded':'SILO'},
                remapping={'target_pose':'silo_pose'}
                )

        sm_disp = SiloState()
        StateMachine.add('SILO', sm_disp,
                transitions={'succeeded':'REACTOR1_FIND'}
                )

        StateMachine.add('REACTOR1_FIND', CBState(getTargetPose),
                transitions={'succeeded':'REACTOR1_TRAVEL'},
                remapping={
                    'target':'reactor1',
                    'target_pose':'reactor1_pose'
                    }
                )
        StateMachine.add('REACTOR1_TRAVEL', TravelState(),
                transitions={'succeeded':'REACTOR1'},
                remapping={'target_pose':'reactor1_pose'}
                )
        sm_reactor1 = ReactorState()
        StateMachine.add('REACTOR1', sm_reactor1,
                transitions={'succeeded':'ENEMY_FIND'}
                )
        # TODO Change back to REACTOR2_FIND


        StateMachine.add('REACTOR2_FIND', CBState(getTargetPose),
                transitions={'succeeded':'REACTOR2_TRAVEL'},
                remapping={
                    'target':'reactor2',
                    'target_pose':'reactor2_pose'
                    }
                )
        StateMachine.add('REACTOR2_TRAVEL', TravelState(),
                transitions={'succeeded':'REACTOR2'},
                remapping={'target_pose':'reactor2_pose'}
                )
        sm_reactor2 = ReactorState()
        StateMachine.add('REACTOR2', sm_reactor2,
                transitions={'succeeded':'REACTOR3_FIND'}
                )

        StateMachine.add('REACTOR3_FIND', CBState(getTargetPose),
                transitions={'succeeded':'REACTOR3_TRAVEL'},
                remapping={
                    'target':'reactor3',
                    'target_pose':'reactor3_pose'
                    }
                )
        StateMachine.add('REACTOR3_TRAVEL', TravelState(),
                transitions={'succeeded':'REACTOR3'},
                remapping={'target_pose':'reactor3_pose'}
                )
        sm_reactor3 = ReactorState()
        StateMachine.add('REACTOR3', sm_reactor3,
                transitions={'succeeded':'ENEMY_FIND'}
                )

        StateMachine.add('ENEMY_FIND', CBState(getTargetPose),
                transitions={'succeeded':'ENEMY_TRAVEL'},
                remapping={
                    'target':'enemy',
                    'target_pose':'enemy_pose'
                    }
                )
        StateMachine.add('ENEMY_TRAVEL', TravelState(),
                transitions={'succeeded':'ENEMY'},
                remapping={'target_pose':'enemy_pose'}
                )
        sm_enemy = EnemyWallState()
        StateMachine.add('ENEMY', sm_enemy,
                transitions={'succeeded':'succeeded'}
                )


    sm_root.execute()


if __name__=='__main__':
    main()
