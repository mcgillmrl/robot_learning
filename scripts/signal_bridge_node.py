#!/usr/bin/env python
'''
Node that takes care of triggering the appropriate signals and service calls.
This is used to deal with resetting the robot in episodic tasks.
'''

import rospy


from rospy.exceptions import ROSException
from std_msgs.msg import Empty, String
from std_srvs.srv import Empty as EmptySrv
from std_srvs.srv import EmptyResponse as EmptySrvResponse


class SignalBridgeNode(object):
    def __init__(self):
        self.name = rospy.get_name()
        # init publishers, subscribers and services
        self.ros_init()
        # notify user
        rospy.loginfo('%s: initialized' % self.name)

    def ros_init(self):
        # create services
        rospy.loginfo('%s: starting reset robot service' % self.name)
        self.reset_robot_srv = rospy.Service(
          '/rl/reset_robot', EmptySrv, self.reset_robot)
        rospy.loginfo('%s: starting reset stop service' % self.name)
        self.stop_robot_srv = rospy.Service(
          '/rl/stop_robot', EmptySrv, self.stop_robot)

    def reset_robot(self, req):
        rospy.loginfo("%s: resetting plant" % self.name)
        return EmptySrvResponse()

    def stop_robot(self, req):
        rospy.loginfo("%s: stopping plant" % self.name)
        return EmptySrvResponse()

    def spin(self):
        rospy.spin()


class GazeboSignalBridgeNode(SignalBridgeNode):
    def __init__(self):
        super(GazeboSignalBridgeNode, self).__init__()
        self.name += '_gazebo'
    
    def ros_init(self):
        # wait for services
        rospy.loginfo(
            '%s: waiting for /gazebo/reset_world...' % self.name)
        rospy.wait_for_service('/gazebo/reset_world')
        self.reset_gz_world = rospy.ServiceProxy(
            "/gazebo/reset_world", EmptySrv)

        rospy.loginfo(
            '%s: waiting for /gazebo/pause_physics...' % self.name)
        rospy.wait_for_service('/gazebo/pause_physics')
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', EmptySrv)

        rospy.loginfo(
            '%s: waiting for /gazebo/unpause_physics...' % self.name)
        rospy.wait_for_service('/gazebo/unpause_physics')
        self.unpause = rospy.ServiceProxy(
            '/gazebo/unpause_physics', EmptySrv)
        self.unpause()
        super(GazeboSignalBridgeNode, self).ros_init()

    def reset_robot(self, req):
        try:
            self.reset_gz_world()
            self.unpause()
        except rospy.ServiceException, e:
            rospy.logerr(
                "%s: service call(s) failed in reset_robot: %s" % (self.name,
                                                                   e))                                                           
        return EmptySrvResponse()

    def stop_robot(self, req):
        try:
            rospy.sleep(4.0)
            self.pause()
        except rospy.ServiceException, e:
            rospy.logerr(
                "%s: service call(s) failed in stop_robot: %s" % (name, e))
        return EmptySrvResponse()


class MarshallSignalBridgeNode(SignalBridgeNode):
    def __init__(self):
        self.has_trigger_start = False
        self.marshall_mode = ''
        super(MarshallSignalBridgeNode, self).__init__()
        self.name += '_marshall'

    def ros_init(self):
        rospy.loginfo(
            '%s: waiting for /rl_marshall/set_mode...' % self.name)
        rospy.wait_for_service('/rl_marshall/set_mode')

        self.trigger_reset_pub = rospy.Publisher(
            '/rl/trigger_reset', Empty, queue_size=10)
        self.trigger_stop_pub = rospy.Publisher(
            '/rl/trigger_stop', Empty, queue_size=10)
        self.trigger_start_sub = rospy.Subscriber(
            '/rl/trigger_start', Empty, self.callback_to_trigger_start)
        self.marshall_mode_sub = rospy.Subscriber(
            '/rl_marshall/mode', String, self.callback_to_marshall_mode)
        super(MarshallSignalBridgeNode, self).ros_init()

    def stop_robot(self, req):
        self.trigger_stop_pub.publish()
        while self.marshall_mode == 'rl':
            rospy.sleep(0.01)
        return EmptySrvResponse()

    def reset_robot(self, req):
        rospy.loginfo("%s: resetting plant" % rospy.get_name())
        self.has_trigger_start = False
        self.trigger_reset_pub.publish()  # tell marshall to prompt user
        msg_ = "%s: in reset_plant, waiting for /rl/trigger_start..."
        rospy.loginfo(msg_ % rospy.get_name())
        # wait till received /aqua_rl/trigger_start (from joy/user)
        while not self.has_trigger_start and not rospy.is_shutdown():
            rospy.sleep(0.1)

        return EmptySrvResponse()

    def callback_to_trigger_start(self, msg):
        self.has_trigger_start = True

    def callback_to_marshall_mode(self, msg):
        self.marshall_mode = msg.data


if __name__ == "__main__":
    rospy.init_node('signal_bridge_node')

    is_sim = True
    if rospy.has_param('~is_sim'):
        is_sim = rospy.get_param('~is_sim')
    else:  # attempt to automatically determine if gazebo is present
        try:
            name = rospy.get_name()
            rospy.loginfo(
                '%s: auto-setting is_sim via /gazebo/reset_world...' % name)
            rospy.wait_for_service('/gazebo/reset_world', timeout=10.0)
        except ROSException:
            is_sim = False

    if is_sim:
        rospy.loginfo("Initializing Gazebo signal bridge")
        node = GazeboSignalBridgeNode()
    else:
        rospy.loginfo("Initializing marshall signal bridge")
        node = MarshallSignalBridgeNode()

    rospy.loginfo('%s: is_sim <- %d' % (node.name, is_sim))

    try:
        node.spin()
    except rospy.ROSInterruptException:
        pass
