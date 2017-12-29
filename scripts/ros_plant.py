#!/usr/bin/env python
import numpy as np
import rospy

from gym import spaces
from Queue import Queue

from robot_learning.msg import ExperienceData
from robot_learning.srv import T2VInfo

from kusanagi.shell import Plant, cost, cartpole


class ROSPlant(Plant):
    '''
    Class for collecting msg and executing policies on a ROS-enabled robot
    '''

    reset_plant_name = '/rl/reset_plant'
    stop_plant_name = '/rl/stop_plant'

    command_dims_srv = '/rl/command_dims'
    state_dims_srv = '/rl/state_dims'

    def __init__(self, state0_dist=None, loss_func=None, dt=1.0,
                 noise=None, name='ROSPlant', init_ros_node=False,
                 *args, **kwargs):
        # init super class
        super(ROSPlant, self).__init__(
            dt=dt, noise_dist=None, angle_dims=[], name=name,
            *args, **kwargs)
        
        # init queue. This is to ensure that we collect experience at every dt
        # seconds
        self.experience_queue = Queue()
        self.t = 0

        # initial state. only needed for gazebo environments, or in cases
        # where we know how to reset to an initial state, (e.g. a robotic arm)
        self.state0_dist = state0_dist

        # reward/loss function. If not specified, the step function will
        # return None for the reward/loss function
        if loss_func is not None:
            self.loss_func = cost.build_loss_func(
                loss_func, False,self.name+'_loss')
        else:
            self.loss_func = None
        
        # initialize ros node, publishers and subscribers
        self.ros_init(init_ros_node)

        # Observation and action spaces will be populated by reading from the
        # appropriate topics
        self.init_obs_act_spaces()
        
        # initialize publishers and subscribers
        self.command_pub = rospy.Publisher(
            '/rl/command_data', ExperienceData, queue_size=-1)
        self.experience_sub = rospy.Subscriber(
            '/rl/experience_data', ExperienceData, self.experience_callback,
            queue_size=-1)
        
        # get initial state
        rospy.loginfo(
            '[%s] waiting for first experience data msg...' % (self.name))
        print self.t
        self.t, self.state, self.cmd = self.wait_for_state(self.dt)
        rospy.loginfo('[%s] Ready.'%(self.name))
    
    def ros_init(self, init_ros_node=False):
        # init plant ros node
        if init_ros_node:
            rospy.init_node(self.name, anonymous=True, disable_signals=True)
        # init time
        self.t = rospy.get_time()

    def init_obs_act_spaces(self):
        rospy.loginfo(
            '[%s] waiting for %s...' % (self.name, ROSPlant.command_dims_srv))
        rospy.wait_for_service(ROSPlant.command_dims_srv)
        cdims = rospy.ServiceProxy(ROSPlant.command_dims_srv, T2VInfo)
        rospy.loginfo(
            '[%s] waiting for %s...' % (self.name, ROSPlant.state_dims_srv))
        rospy.wait_for_service(ROSPlant.state_dims_srv)
        sdims = rospy.ServiceProxy(ROSPlant.state_dims_srv, T2VInfo)
        
        # TODO get min max ranges from config file or from service
        o_lims = np.array([1e3 for i in range(sdims().value)])
        self.observation_space = spaces.Box(-o_lims, o_lims)
        a_lims = np.array([1e3 for i in range(cdims().value)])
        self.action_space = spaces.Box(-a_lims, a_lims)        

    def experience_callback(self, msg):
        # put incoming messages into experience queue
        q = self.experience_queue
        if q.full():
            q.get()
        t = msg.header.stamp.secs + msg.header.stamp.nsecs*1e-9
        state = np.array(msg.state_data)
        cmd = np.array(msg.command_data)
        q.put((t, state, cmd))

    def wait_for_state(self, dt=None):
        if dt is None:
            dt = self.dt
        t1 = self.t + dt
        t = self.t
        state = []
        cmd = []
        while t < t1:
            if self.experience_queue.empty():
                # sleep for a short time
                rospy.sleep(0.1*dt)
            else:
                t, state, cmd = self.experience_queue.get(
                    timeout=0.01*dt)
        return t, state, cmd

    def apply_control(self, u):
        '''
            publish control message. We send the
        '''
        self.u = u
        msg = ExperienceData()
        msg.header.stamp = rospy.Time()
        # we fill the state msg for logging purposes, the topics to vector
        # node ignores this information.
        msg.state_data = self.state.tolist()
        msg.command_data = u.tolist()
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
        t, state, cmd = self.wait_for_state(self.dt)
        
        # save latest measurement info
        self.state = state
        self.t = t
        info['t'] = self.t
        info['action'] = action

        # evaluate cost, if given
        cost = None
        if self.loss_func is not None:
            cost = self.loss_func(self.state)

        # return output following the openai gym convention    
        return self.state, cost, False, info
    
    def _reset(self):
        ''' 
            calls the registered reset service with the desired state. 
        '''
    
    def _close(self):
        '''
            class any registered service with empty messages
        '''
        pass
    
