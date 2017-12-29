#!/usr/bin/env python2

import argparse
import rospy
import yaml
from ros_plant import ROSPlant

from kusanagi.base import apply_controller
from kusanagi.ghost.control import NNPolicy, RandPolicy

def parse_config(config_path):
    '''
        loads configuration for learning tasks (policy and costs parameters)
        from a yaml file
    '''
    with open(config_path, 'r') as f:
        config_data = yaml.load(f)
        print config_data
        return config_data

def run_client():
    pass

if __name__=='__main__':
    parser = argparse.ArgumentParser(
        'rosrun robot_learning task_client.py')
    parser.add_argument(
        'config_path', metavar='FILE',
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
    rospy.init_node('kusanagi_ros')

    # import yaml
    
    config = parse_config(args.config_path)
    print config



    # init plant
    #env = ROSPlant()

    # init policy
    #pol = NNPolicy(sum(env.observation_space.shape), maxU=[])
