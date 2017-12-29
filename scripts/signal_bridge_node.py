#!/usr/bin/env python
'''
Node that takes care of triggering the appropriate signals and service calls.abs
This is used to deal with resetting the robot in episodic tasks.abs
TODO remove references to the aqua robot from here
'''

import sys
import rospy

from rospy.exceptions import ROSException
from std_msgs.msg import Empty, String
from std_srvs.srv import Empty as EmptySrv
from std_srvs.srv import EmptyResponse as EmptySrvResponse
from gazebo_msgs.srv import SetModelState

class MBRLSignalBridgeNode:
  def __init__(self):
    self.has_trigger_start = False
    self.marshall_mode = ''
    
    rospy.init_node('mbrl_signal_bridge_node')
    
    self.is_sim = True
    if rospy.has_param('~is_sim'):
      self.is_sim = rospy.get_param('~is_sim')
    else: # attempt to automatically determine if gazebo is present
      try:
        rospy.loginfo('%s: auto-setting is_sim via /gazebo/reset_world...'%rospy.get_name())
        rospy.wait_for_service('/gazebo/reset_world', timeout=5.0)
      except ROSException, err:
        self.is_sim = False
    rospy.loginfo('%s: is_sim <- %d' % (rospy.get_name(), self.is_sim))
    
    # setup service clients for simulator
    if self.is_sim: # for simulated robot, without marshall
      # wait for services
      rospy.loginfo('%s: waiting for /gazebo/reset_world...' % rospy.get_name())
      rospy.wait_for_service('/gazebo/reset_world')
      self.reset_aqua_world = rospy.ServiceProxy("/gazebo/reset_world", EmptySrv)
      
      rospy.loginfo('%s: waiting for /gazebo/pause_physics...' % rospy.get_name())
      rospy.wait_for_service('/gazebo/pause_physics')
      self.pause = rospy.ServiceProxy('/gazebo/pause_physics', EmptySrv)
      
      rospy.loginfo('%s: waiting for /gazebo/pause_physics...' % rospy.get_name())
      rospy.wait_for_service('/gazebo/unpause_physics')
      self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', EmptySrv)
    
    # for real robot via marshall
    else: # not self.is_sim
      rospy.loginfo('%s: waiting for /aqua_marshall/set_mode (to ensure marshall initialized)...' % rospy.get_name())
      rospy.wait_for_service('/aqua_marshall/set_mode')
      
      self.trigger_reset_pub = rospy.Publisher('/mbrl/trigger_reset', Empty, queue_size=10)
      self.trigger_stop_pub = rospy.Publisher('/mbrl/trigger_stop', Empty, queue_size=10)
      self.trigger_start_sub = rospy.Subscriber('/mbrl/trigger_start', Empty, self.callback_to_trigger_start)
      self.marshall_mode_sub = rospy.Subscriber('/aqua_marshall/mode', String, self.callback_to_marshall_mode)

    # create services
    self.reset_plant_svr = rospy.Service('/mbrl/reset_plant', EmptySrv, self.callback_to_reset_plant)
    self.stop_plant_svr = rospy.Service('/mbrl/stop_plant', EmptySrv, self.callback_to_stop_plant)
    
    # notify user
    rospy.loginfo('%s: initialized' % rospy.get_name())
    
    
  def callback_to_reset_plant(self, req):
    rospy.loginfo("%s: resetting plant" % rospy.get_name())
    
    if self.is_sim:
      try:
        self.reset_aqua_world()
        self.unpause()
      except rospy.ServiceException, e:
        rospy.logerr("%s: service call(s) failed in reset_plant: %s" % (rospy.get_name(), e))

    else: # not self.is_sim
      self.has_trigger_start = False
      self.trigger_reset_pub.publish() # tell marshall to prompt user
      rospy.loginfo("%s: in reset_plant, waiting for /mbrl/trigger_start..." % rospy.get_name())
      while not self.has_trigger_start and not rospy.is_shutdown(): # wait till received /mbrl/trigger_start (from joy/user)
        rospy.sleep(0.1)
        
    return EmptySrvResponse()


  def callback_to_stop_plant(self, req):
    rospy.loginfo("%s: stopping plant" % rospy.get_name())
    if self.is_sim:
      try:
        self.pause()
      except rospy.ServiceException, e:
        rospy.logerr("%s: service call(s) failed in stop_plant: %s" % (rospy.get_name(), e))
        
    else: # not self.is_sim
      self.trigger_stop_pub.publish()
      while self.marshall_mode == 'rl':
        rospy.sleep(0.01)
      
    return EmptySrvResponse()


  def callback_to_trigger_start(self, msg):
    self.has_trigger_start = True
    

  def callback_to_marshall_mode(self, msg):
    self.marshall_mode = msg.data


  def spin(self):
    rospy.spin()


if __name__ == "__main__":
  try:
    node = MBRLSignalBridgeNode()
    node.spin()
  except rospy.ROSInterruptException:
    pass
