#!/usr/bin/env python2
'''
    Client code for collecting data with a robot in an episodic manner.
    Triggers model learning when new data is available, and policy learning
    whe new model is available
'''
import argparse
import numpy as np
import os
import rospy
import threading
import yaml
import requests
import pickle

from collections import OrderedDict
from functools import partial
from Queue import Queue, Empty
from ros_plant import ROSPlant

from kusanagi.base import (apply_controller, train_dynamics,
                           preprocess_angles, ExperienceDataset)
from kusanagi.ghost.algorithms import mc_pilco
from kusanagi.ghost.control import RandPolicy
from kusanagi import utils

compile_lock = threading.RLock()

def numpy_code_constructor(loader, node):
    code_string = loader.construct_scalar(node)
    return eval(code_string)


def include_constructor(loader, node):
    filename = loader.construct_scalar(node)
    if not os.path.isabs(filename):
        root = os.path.dirname(loader.stream.name)
        filename = os.path.abspath(os.path.join(root, filename))
    data = {}
    with open(filename, 'r') as f:
        data = yaml.load(f)
    return data


def default_config():
    config = dict(
        initial_random_trials=4,
        output_directory='/data/robot_learning',
    )
    return config


def parse_config(config_path):
    '''
        loads configuration for learning tasks (policy and costs parameters)
        from a yaml file
    '''
    yaml.add_constructor('!include', include_constructor)
    yaml.add_constructor('!numpy', numpy_code_constructor)
    config = default_config()
    with open(config_path, 'r') as f:
        config = yaml.load(f)
    return config


if __name__ == '__main__':
    np.set_printoptions(linewidth=200, precision=3)
    rospy.init_node('kusanagi_ros', disable_signals=True)

    parser = argparse.ArgumentParser(
        'rosrun robot_learning task_client.py')
    parser.add_argument(
        'config_path', metavar='FILE',
        help='A YAML file containing the configuration for the learning task.',
        type=str)

    # args = parser.parse_args()
    args = parser.parse_args(rospy.myargv()[1:])
    load_experience = True

    # import yaml
    config = parse_config(args.config_path)

    # init output dir
    output_directory = config['output_directory']
    utils.set_output_dir(output_directory)
    try:
        os.mkdir(output_directory)
    except Exception as e:
        if not load_experience:
            # move the old stuff
            dir_time = str(os.stat(output_directory).st_ctime)
            target_dir = os.path.dirname(output_directory)+'_'+dir_time
            os.rename(output_directory, target_dir)
            os.mkdir(output_directory)
            utils.print_with_stamp(
                'Moved old results from [%s] to [%s]' % (output_directory,
                                                         target_dir))

    utils.print_with_stamp('Results will be saved in [%s]' % output_directory)

    # init environment with first task params
    plant_params = config['tasks'].values()[0]['plant']
    env = ROSPlant(**plant_params)

    # init task queue and list of learning threads
    tasks = Queue()
    task_state = {}
    polopt_threads = []

    # populate task queue
    for task_name in config['tasks']:
        task_state[task_name] = 'init'
        spec = config['tasks'][task_name]
        exp = spec.get('experience', None)
        pol = spec['policy']
        pol.evaluate(np.zeros(pol.D))
        random_exp_path = spec.get('random_exp_path', None)
        if exp is None:
            exp = ExperienceDataset(name=task_name)
            if not exp.load() and random_exp_path is not None:
                base_path, filename = os.path.split(
                    random_exp_path)
                fname = exp.filename
                exp.load(base_path, filename)
                # restore previous filename
                exp.filename = fname

            if exp.n_episodes() > 0:
                if len(exp.policy_parameters[-1]) > 0:
                    pol.set_params(exp.policy_parameters[-1])
                spec['initial_random_trials'] -= exp.n_episodes()
                task_state[task_name] = 'ready'
        spec['experience'] = exp
        n_rnd = len([p for p in exp.policy_parameters if len(p) == 0])
        spec['current_iteration'] = n_rnd
        tasks.put((task_name, spec))


    # while tasks are not done
    while not all([st == 'done' for st in task_state]):
        # get new task
        new_task_ready = False
        rospy.loginfo('Waiting for new task')
        while not new_task_ready:
            try:
                name, spec = tasks.get(timeout=5)
                new_task_ready = True
            except Empty:
                pass
        #utils.set_logfile("%s.log" % name, base_path="/localdata")
        # if task is done, pass
        exp = spec.get('experience')
        n_rnd = len([p for p in exp.policy_parameters if len(p) == 0])
        if task_state[name] == 'done':
            rospy.loginfo(
                'Finished %s task [iteration %d]' % (
                    name, spec['current_iteration']-n_rnd+1))
            continue
        msg_ = '==== Executing %s task [iteration %d] ====' % (
            name, spec['current_iteration']-n_rnd+1)
        rospy.loginfo(msg_)
        utils.print_with_stamp(msg_)

        # set plant parameters for current task
        plant_params = spec['plant']
        env.init_params(**plant_params)

        # load policy
        pol_params = exp.policy_parameters[spec['current_iteration']]
        if len(pol_params) == 0:
            # collect random experience
            pol = RandPolicy(maxU=spec['policy'].maxU,
                             random_walk=spec.get('random_walk', False))
        else:
            # TODO load policy parameters from disk
            pol = spec['policy']
            pol.set_params(pol_params)
        # set task horizon
        H = int(np.ceil(spec['horizon_secs']/env.dt))

        # execute tasks and collect experience data
        preprocess = None
        if hasattr(pol, 'angle_dims'):
            preprocess = partial(
                preprocess_angles, angle_dims=pol.angle_dims)
        apply_controller(env, pol, H, preprocess)

        spec['current_iteration'] += 1
        tasks.put((name, spec))

