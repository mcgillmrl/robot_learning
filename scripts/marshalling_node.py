#!/usr/bin/env python
import math
import rospy
from std_msgs.msg import Empty, String
from std_srvs.srv import Empty as EmptySrv
from std_srvs.srv import EmptyResponse as EmptySrvResponse
from sensor_msgs.msg import Joy
from aquacore.msg import PeriodicLegCommand
from aquacore.srv import SetAutopilotMode, SetString, SetStringResponse
from JoyState import JoyState


class AquaMarshallNode:
  def __init__(self):
    rospy.init_node('rl_marshall_node')
    
    self.FSM = 'ap'  # possible values: 'ap'/'rl'/'ap_prompt'
    self.prev_joy_state = None

    self.plc_out_pub = rospy.Publisher('/aqua/periodic_leg_command', PeriodicLegCommand, queue_size=10)
    self.fsm_pub = rospy.Publisher('/rl_marshall/mode', String,queue_size=10, latch=True)
    self.trigger_start_pub = rospy.Publisher('/aqua_rl/trigger_start', Empty, queue_size=10)
    self.trigger_stop_pub = rospy.Publisher('/aqua_rl/trigger_stop', Empty, queue_size=10)
    
    rospy.loginfo('%s: waiting for /aqua/set_3Dauto_mode...' % rospy.get_name())
    rospy.wait_for_service('/aqua/set_3Dauto_mode')
    self.set_ap_mode_cln = rospy.ServiceProxy('/aqua/set_3Dauto_mode', SetAutopilotMode)

    rospy.loginfo('%s: waiting for /aqua/reset_3D_autopilot_state...' % rospy.get_name())
    rospy.wait_for_service('/aqua/reset_3D_autopilot_state')
    self.reset_ap_cln = rospy.ServiceProxy('/aqua/reset_3D_autopilot_state', EmptySrv)

    self.set_mode_svr = rospy.Service('/rl_marshall/set_mode', SetString, self.handle_set_mode)
    self.trigger_start_sub = rospy.Subscriber('/aqua_rl/trigger_start', Empty, self.handle_trigger_start)
    self.trigger_stop_sub = rospy.Subscriber('/aqua_rl/trigger_stop', Empty, self.handle_trigger_stop)
    self.trigger_reset_sub = rospy.Subscriber('/aqua_rl/trigger_reset', Empty, self.handle_trigger_reset)
    self.plc_in_ap_sub = rospy.Subscriber('/sandbox/AP/periodic_leg_command', PeriodicLegCommand, self.handle_ap_plc)
    self.plc_in_rl_sub = rospy.Subscriber('/sandbox/RL/periodic_leg_command', PeriodicLegCommand, self.handle_rl_plc)
    self.joy_sub = rospy.Subscriber('/joy', Joy, self.handle_joy)
    
    if self.FSM == 'ap' or self.FSM == 'ap_prompt':
      self.activate_ap()
    else:
      self.deactivate_ap()
    self.fsm_pub.publish(String(self.FSM))
    
    rospy.loginfo('%s: initialized into %s mode' % (rospy.get_name(), self.FSM))

    
  def activate_ap(self):
    self.reset_ap_cln()
    self.set_ap_mode_cln(4) # depth-regulated
    

  def deactivate_ap(self):
    self.reset_ap_cln()
    self.set_ap_mode_cln(0)
    self.reset_ap_cln()
    
    # Force zero-center on all flippers
    msg = PeriodicLegCommand()
    self.plc_out_pub.publish(msg)
    rospy.sleep(1.0)
  
  
  def handle_ap_plc(self, msg):
    if self.FSM == 'ap' or self.FSM == 'ap_prompt':
      self.plc_out_pub.publish(msg)


  def handle_rl_plc(self, msg):
    if self.FSM == 'rl':
      self.plc_out_pub.publish(msg)


  def handle_joy(self, msg):
    # Parse msg
    joy_state = JoyState()
    if not joy_state.fromJoyMsg(msg):
      rospy.logwarn('%s: ignoring joy msg with %d axes and %d buttons' % (rospy.get_name(), len(msg.axes), len(msg.buttons)))
      return
      
    # Set default prev state
    if self.prev_joy_state is None:
      self.prev_joy_state = joy_state

    # Handle E_STOP request (Select held down + B pressed)
    if joy_state.Select and not self.prev_joy_state.B and self.prev_joy_state.B:
      rospy.loginfo('%s: handling E_STOP request' % rospy.get_name())
      self.set_mode('ap')

    # Handle FORCE_AP request (Select held down + Start released)
    elif joy_state.Select and self.prev_joy_state.Start and not joy_state.Start:
      if self.FSM == 'rl':
        rospy.loginfo('%s: handling FORCE_AP request' % rospy.get_name())
        self.trigger_stop_pub.publish()

    # Handle TRIGGER_START request
    elif self.prev_joy_state.Start and not joy_state.Start:
      if self.FSM == 'ap_prompt':
        rospy.loginfo('%s: handling TRIGGER_START request' % rospy.get_name())
        self.trigger_start_pub.publish()
      
    self.prev_joy_state = joy_state


  def handle_trigger_reset(self, msg):
    if self.FSM == 'ap':
      self.set_mode('ap_prompt')
    else:
      rospy.logwarn('%s: ignoring TRIGGER_RESET in %s mode' % (rospy.get_name(), self.FSM))


  def handle_trigger_start(self, msg):
    if self.FSM == 'ap_prompt':
      self.set_mode('rl')
    else:
      rospy.logwarn('%s: ignoring TRIGGER_START in %s mode' % (rospy.get_name(), self.FSM))
  
  
  def handle_trigger_stop(self, msg):
    if self.FSM == 'rl':
      self.set_mode('ap')
    else:
      rospy.logwarn('%s: ignoring TRIGGER_STOP in %s mode' % (rospy.get_name(), self.FSM))
  
  
  def handle_set_mode(self, req):
    self.set_mode(req.value)
    return SetStringResponse()


  def set_mode(self, newFSM): # 'rl'/'ap'/'ap_prompt'
    if not (newFSM == 'rl' or newFSM == 'ap' or newFSM == 'ap_prompt'):
      rospy.logerr('%s: set_mode arg=%s unrecognized; expecting rl|ap|ap_prompt' % (rospy.get_name(), newFSM))
      return
      
    if newFSM != self.FSM:
      if self.FSM == 'rl' and (newFSM == 'ap' or newFSM == 'ap_prompt'):
        self.activate_ap()
      elif (self.FSM == 'ap' or self.FSM == 'ap_prompt') and newFSM == 'rl':
        self.deactivate_ap() # also zero-centers all flippers for 1s before returning
      self.FSM = newFSM
      self.fsm_pub.publish(String(self.FSM))
      rospy.loginfo('%s: set to %s mode' % (rospy.get_name(), self.FSM))
      

  def spin(self):
    rospy.spin()



if __name__ == '__main__':
  try:
    node = AquaMarshallNode()
    node.spin()
  except rospy.ROSInterruptException: pass
