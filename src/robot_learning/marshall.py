#!/usr/bin/env python
import rospy
from std_msgs.msg import Empty, String
from robot_learning.srv import SetString, SetStringResponse

from enum import IntEnum


class FSM_STATES(IntEnum):
    USER = 1
    RL = 2
    USER_PROMPT = 3


class MarshallNode(object):
    def __init__(self, name='rl_marshall'):
        rospy.init_node(name)
        self.FSM = FSM_STATES.USER

        self.set_mode_svr = rospy.Service(
            '/rl_marshall/set_mode', SetString, self.handle_set_mode)
        self.fsm_pub = rospy.Publisher(
            '/rl_marshall/mode', String, queue_size=10, latch=True)
        self.trigger_start_pub = rospy.Publisher(
            '/rl/trigger_start', Empty, queue_size=10)
        self.trigger_stop_pub = rospy.Publisher(
            '/rl/trigger_stop', Empty, queue_size=10)
        self.trigger_start_sub = rospy.Subscriber(
            '/rl/trigger_start', Empty, self.handle_trigger_start)
        self.trigger_stop_sub = rospy.Subscriber(
            '/rl/trigger_stop', Empty, self.handle_trigger_stop)
        self.trigger_reset_sub = rospy.Subscriber(
            '/rl/trigger_reset', Empty, self.handle_trigger_reset)

    def set_mode(self, newFSM):
        if not (newFSM in FSM_STATES):
            msg_ = '%s: set_mode arg=%s unrecognized; expecting one of %s'
            rospy.logerr(
                msg_ % (rospy.get_name(), newFSM, FSM_STATES._member_names_))
            return

        if newFSM != self.FSM:
            if self.FSM == FSM_STATES.RL and (newFSM == FSM_STATES.USER or newFSM == FSM_STATES.USER_PROMPT):
                self.set_user_mode()
            elif (self.FSM == FSM_STATES.USER or self.FSM == FSM_STATES.USER_PROMPT) and newFSM == FSM_STATES.RL:
                self.set_rl_mode()
            rospy.loginfo('%s: set to %s mode' % (rospy.get_name(), self.FSM))
            self.fsm_pub.publish(str(self.FSM))
            self.FSM = newFSM

    def handle_trigger_reset(self, msg):
        if self.FSM == FSM_STATES.USER:
            self.set_mode(FSM_STATES.USER_PROMPT)
        else:
            rospy.logwarn(
              '%s: ignoring TRIGGER_RESET in %s mode' % (
                  rospy.get_name(), self.FSM))

    def handle_trigger_start(self, msg):
        if self.FSM == FSM_STATES.USER_PROMPT:
            self.set_mode(FSM_STATES.RL)
        else:
            rospy.logwarn(
              '%s: ignoring TRIGGER_START in %s mode' % (
                  rospy.get_name(), self.FSM))

    def handle_trigger_stop(self, msg):
        if self.FSM == FSM_STATES.RL:
            self.set_mode(FSM_STATES.USER)
        else:
            rospy.logwarn('%s: ignoring TRIGGER_STOP in %s mode' % (
                rospy.get_name(), self.FSM))

    def handle_set_mode(self, req):
        val = getattr(FSM_STATES, req.value)
        self.set_mode(val)
        return SetStringResponse()

    def set_user_mode(self):
        raise NotImplementedError

    def set_rl_mode(self):
        raise NotImplementedError
