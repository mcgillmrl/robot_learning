#!/usr/bin/env python
import gym
import numpy as np
import rospy

from gym import spaces
from collections import deque

from std_srvs.srv import Empty as EmptySrv
from robot_learning.msg import ExperienceData
from robot_learning.srv import T2VInfo
from kusanagi.utils import print_with_stamp

class ROSPlant(gym.Env):
    '''
    Class for collecting msg and executing policies on a ROS-enabled robot
    '''

    reset_srv_name = '/rl/reset_robot'
    stop_srv_name = '/rl/stop_robot'

    command_dims_srv_name = '/rl/command_dims'
    state_dims_srv_name = '/rl/state_dims'

    def __init__(self, state0_dist=None, loss_func=None, dt=0.5,
                 noise_dist=None, angle_dims=[], name='ROSPlant',
                 init_ros_node=False, max_experience_queue_size=2000,
                 command_topic='/rl/command_data',
                 experience_topic='/rl/experience_data',
                 *args, **kwargs):
        # init queue. This is to ensure that we collect experience at every dt
        # seconds
        self.t = rospy.get_time()
        self.experience_queue = deque(maxlen=max_experience_queue_size)

        # initalize internal plant parameteers
        self.init_params(state0_dist, loss_func, dt, noise_dist, angle_dims,
                         name, *args, **kwargs)

        # initialize ros node, publishers and subscribers
        self.ros_init(init_ros_node)

        # Observation and action spaces will be populated by reading from the
        # appropriate topics
        self.init_obs_act_spaces()

        # initialize publishers and subscribers
        self.command_pub = rospy.Publisher(
            command_topic, ExperienceData, queue_size=-1)
        self.experience_sub = rospy.Subscriber(
            experience_topic, ExperienceData, self.experience_callback,
            queue_size=1000)

        # get initial state
        rospy.loginfo(
            '[%s] waiting for first experience data msg...' % (self.name))
        self.t, self.state = self.wait_for_state(self.dt)
        rospy.loginfo('[%s] Ready.' % (self.name))

    def init_params(self, state0_dist=None, loss_func=None, dt=0.5,
                    noise_dist=None, angle_dims=[], name='ROSPlant',
                    *args, **kwargs):
        self.name = name
        self.dt = dt
        self.noise_dist = noise_dist
        self.angle_dims = angle_dims

        # initial state. only needed for gazebo environments, or in cases
        # where we know how to reset to an initial state, (e.g. a robotic arm)
        self.state0_dist = state0_dist

        # user specified reward/loss function. takes as input state vector,
        # produces as output scalar reward/cost. If not specified, the step
        # function will return None for the reward/loss function
        self.loss_func = loss_func

    def ros_init(self, init_ros_node=False):
        # init plant ros node
        if init_ros_node:
            rospy.init_node(self.name, anonymous=True, disable_signals=True)
        # start service proxies
        rospy.loginfo(
            '[%s] waiting for %s...' % (self.name, ROSPlant.reset_srv_name))
        rospy.wait_for_service(ROSPlant.reset_srv_name)
        self.reset_srv = rospy.ServiceProxy(ROSPlant.reset_srv_name, EmptySrv)
        rospy.loginfo(
            '[%s] waiting for %s...' % (self.name, ROSPlant.stop_srv_name))
        rospy.wait_for_service(ROSPlant.stop_srv_name)
        self.stop_srv = rospy.ServiceProxy(ROSPlant.stop_srv_name, EmptySrv)

        # init time
        self.t0 = rospy.get_time()
        self.t = self.t0

    def init_obs_act_spaces(self):
        rospy.loginfo(
            '[%s] waiting for %s...' % (self.name,
                                        ROSPlant.command_dims_srv_name))
        rospy.wait_for_service(ROSPlant.command_dims_srv_name)
        cdims = rospy.ServiceProxy(ROSPlant.command_dims_srv_name, T2VInfo)
        rospy.loginfo(
            '[%s] waiting for %s...' % (self.name,
                                        ROSPlant.state_dims_srv_name))
        rospy.wait_for_service(ROSPlant.state_dims_srv_name)
        sdims = rospy.ServiceProxy(ROSPlant.state_dims_srv_name, T2VInfo)

        # TODO get min max ranges from config file or from service
        o_lims = np.array([1e3 for i in range(sdims().value)])
        self.observation_space = spaces.Box(-o_lims, o_lims)
        a_lims = np.array([1e3 for i in range(cdims().value)])
        self.action_space = spaces.Box(-a_lims, a_lims)

    def experience_callback(self, msg):
        # put incoming messages into experience queue
        q = self.experience_queue

        t = (msg.header.stamp.secs - self.t0) + msg.header.stamp.nsecs*1e-9 
        state = msg.state_data
        q.append((t, state))
        # print_with_stamp("%s, %s" % (str(self.t), str(t)), same_line=False)

    def wait_for_state(self, dt=None, slop=1.0e-2):
        if dt is None:
            dt = self.dt
        t1 = self.t + dt
        t = self.t
        t_init = self.t
        q = self.experience_queue
        state = None
        while t < t1:
            if len(q) == 0:
                # sleep for a short time
                rospy.sleep(0.01*dt)
            else:
                t, state = q.popleft()
                t += self.t0
                if t1 - t < slop:
                    # we process messages that are a little bit early as if
                    # they were on the clock at the desired rate, to avoid
                    # accumulating delays
                    t = t1
                    break
        return t, state

    def apply_control(self, u):
        '''
            publish control message. We send the
        '''
        if type(u) is np.ndarray:
            if u.ndim < 1:
                u = u[None]
            u = u.tolist() 
        self.cmd = u
        msg = ExperienceData()
        msg.header.stamp = rospy.Time.now()
        # we fill the state msg for logging purposes, the topics to vector
        # node ignores this information.
        msg.state_data = self.state
        msg.command_data = u
        self.command_pub.publish(msg)

    def _step(self, action):
        # apply action and return state dt seconds after sending command
        # For control tasks, the robot driver should be responsible to decide
        # whether to use zero-order hold (constant command during dt) or
        # any other scheme.
        info = {}

        # first apply control
        self.apply_control(action)

        # step for dt seconds
        t, state = self.wait_for_state(self.dt)

        # save latest measurement info
        self.state = np.array(state)
        self.t = t
        info['t'] = self.t
        info['action'] = action

        # evaluate cost, if given
        cost = None
        if self.loss_func is not None:
            cost = self.loss_func(self.state[None, :])

        # return output following the openai gym convention
        return self.state, cost, False, info

    def _reset(self):
        '''
            calls the registered reset service with the desired state.
        '''
        rospy.loginfo("[%s] Resetting robot..." % (self.name))
        self.reset_srv()
        rospy.loginfo("[%s] Done! Waiting for state update..." % (self.name))
        # init time
        self.t = rospy.get_time()
        self.t0 = self.t
        self.t, self.state = self.wait_for_state(dt=0.1*self.dt)
        self.t0 = self.t
        rospy.loginfo("[%s] Robot ready" % (self.name))
        return self.state

    def _close(self):
        '''
            class any registered service with empty messages
        '''
        pass

    def stop(self):
        '''
            calls the registered reset service with the desired state.
        '''
        self.stop_srv()
