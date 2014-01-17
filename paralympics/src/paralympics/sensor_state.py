import roslib; roslib.load_manifest('smach_ros')
import rospy

import threading
import traceback

from smach import *

__all__ = ['SensorState']

class SensorState(State):
    def __init__(self, topic, msg_type, timeout, loopFn=None,
            outcomes=['succeeded', 'preempted', 'aborted'],
            input_keys=[], output_keys=[]):

        State.__init__(self,
                outcomes=outcomes,
                input_keys=input_keys + ['msg_in'],
                output_keys=output_keys + ['msg_out']
                )

        self._loop = loopFn
        self._timeout = timeout
        self._trigger_cond = threading.Condition()
        self._topic = topic
        self._msg_type = msg_type
        self._msg = None


    def execute(self, ud):
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
                ret = self.loop(msg, ud)
                if ret:
                    sub.unregister()
                    ud.msg_out = msg
                    return ret

                self._msg = None
                msg = None


    def loop(self, msg, ud):
        self._loop(msg, ud)

    def _msg_cb(self, msg):
        self._msg = msg

        self._trigger_cond.acquire()
        self._trigger_cond.notify()
        self._trigger_cond.release()
