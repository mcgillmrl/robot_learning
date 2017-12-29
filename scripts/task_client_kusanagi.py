#!/usr/bin/env python2
'''
Task Client:
Manages execution of policies on the target system and
data collection
'''
import argparse
from Queue import Queue

from ros_plant import ROSPlant

from kusanagi.base import apply_controller
from kusanagi.ghost import control


class Task(object):
    def __init__(self, name, policy):
        self.name = name

def parse_config_file(path):
    '''
    Returns a dictionary with the configuration needed
    for applying controls
    '''
    return {}

def init_env(config):
    '''
    Initializes the data structures needed for the learning tasks
    '''
    # init target environment
    env = ROSPlant()
    
    print (env.observation_space)

    # init policy (params from config file)

    # put initial tasks in the queue


def init_tasks(config):
    pass

def main_loop(config, tasks=[]):
    # we will execute tasks sequentially on the robot. Once a task is done
    # executing, we will launch a thread for asynchronous policy optimization.
    # The results from the optimization will be pushed to a task queue.
    tasks = Queue()
    tasks_done = False

    # initialize plant and policy. Also
    env, pol = init_env(config, task_queue)

    r = rospy.rate(1.0)
    while not tasks_done:
        task = tasks.get()
        r.sleep()
        rospy.loginfo(env.observation_space)

def playback_loop(config, tasks=[]):
    pass


if __name__=='__main__':
    parser = argparse.ArgumentParser(
        'rosrun robot_learning task_client.py')
    parser.add_argument(
        'config_file', metavar='FILE',
        help='A YAML file containing the configuration for the learning task.',
        type=str)
    parser.add_argument(
        '-p', '--playback', help='Whether to run learnedpolicies only',
        action='store_true')
    parser.add_argument(
        '-t', '--tasks',
        help='Tasks from the configuration file to be executed. Default is all.',
        type=str, nargs='+', default=[])
    args = parser.parse_args()

    config = parse_config_file(args.config_file)

    main_loop(config)
