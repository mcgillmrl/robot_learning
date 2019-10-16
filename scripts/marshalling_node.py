#!/usr/bin/env python
import rospy

from std_srvs.srv import Empty as EmptySrv
from sensor_msgs.msg import Joy
from aquacore.srv import SetAutopilotMode
from aquacore.msg import PeriodicLegCommand
from JoyState import JoyState

from robot_learning.marshall import MarshallNode, FSM_STATES


class AquaMarshallNode(MarshallNode):
    def __init__(self, name='aqua_marshall'):
        super(AquaMarshallNode, self).__init__(name)
        self.prev_joy_state = None

        self.plc_out_pub = rospy.Publisher(
            '/aqua/periodic_leg_command', PeriodicLegCommand, queue_size=10)
        rospy.loginfo(
            '%s: waiting for /aqua/set_3Dauto_mode...' % rospy.get_name())
        rospy.wait_for_service('/aqua/set_3Dauto_mode')
        self.set_ap_mode_cln = rospy.ServiceProxy(
            '/aqua/set_3Dauto_mode', SetAutopilotMode)
        msg_ = '%s: waiting for /aqua/reset_3D_autopilot_state...'
        rospy.loginfo(msg_ % rospy.get_name())
        rospy.wait_for_service(
            '/aqua/reset_3D_autopilot_state')
        self.reset_ap_cln = rospy.ServiceProxy(
            '/aqua/reset_3D_autopilot_state', EmptySrv)

        self.plc_in_ap_sub = rospy.Subscriber(
            '/sandbox/AP/periodic_leg_command', PeriodicLegCommand,
            self.handle_ap_plc)
        self.plc_in_rl_sub = rospy.Subscriber(
            '/sandbox/RL/periodic_leg_command', PeriodicLegCommand,
            self.handle_rl_plc)

        self.joy_sub = rospy.Subscriber(
            '/joy', Joy, self.handle_joy)

        if self.FSM in [FSM_STATES.USER, FSM_STATES.USER_PROMPT]:
            self.set_user_mode()
        else:
            self.set_rl_mode()
        self.fsm_pub.publish(str(self.FSM))

        rospy.loginfo(
            '%s: initialized into %s mode' % (rospy.get_name(), self.FSM))

    def set_user_mode(self):
        self.reset_ap_cln()
        self.set_ap_mode_cln(4)  # depth-regulated

    def set_rl_mode(self):
        self.reset_ap_cln()
        self.set_ap_mode_cln(0)
        self.reset_ap_cln()

        # Force zero-center on all flippers
        msg = PeriodicLegCommand()
        self.plc_out_pub.publish(msg)
        rospy.sleep(1.0)

    def handle_ap_plc(self, msg):
        if self.FSM in [FSM_STATES.USER, FSM_STATES.USER_PROMPT]:
            self.plc_out_pub.publish(msg)

    def handle_rl_plc(self, msg):
        if self.FSM == FSM_STATES.RL:
            self.plc_out_pub.publish(msg)

    def handle_joy(self, msg):
        # Parse msg
        joy_state = JoyState()
        if not joy_state.fromJoyMsg(msg):
            msg_ = '%s: waiting for /aqua/reset_3D_autopilot_state...'
            rospy.logwarn(
                msg_ % (rospy.get_name(), len(msg.axes), len(msg.buttons)))
            return

        # Set default prev state
        if self.prev_joy_state is None:
            self.prev_joy_state = joy_state

        # Handle E_STOP request (Select held down + B pressed)
        if joy_state.Select and not self.prev_joy_state.B and self.prev_joy_state.B:
            rospy.loginfo('%s: handling E_STOP request' % rospy.get_name())
            self.set_mode(FSM_STATES.USER)

        # Handle FORCE_AP request (Select held down + Start released)
        elif joy_state.Select and self.prev_joy_state.Start and not joy_state.Start:
            if self.FSM == FSM_STATES.RL:
                rospy.loginfo(
                  '%s: handling FORCE_AP request' % rospy.get_name())
                self.trigger_stop_pub.publish()

        # Handle TRIGGER_START request
        elif self.prev_joy_state.Start and not joy_state.Start:
            if self.FSM == FSM_STATES.USER_PROMPT:
                rospy.loginfo(
                  '%s: handling TRIGGER_START request' % rospy.get_name())
                self.trigger_start_pub.publish()

        self.prev_joy_state = joy_state

    def spin(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = AquaMarshallNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass
