#!/usr/bin/env python2
'''
    Client code for collecting data with a robot in an episodic manner.
    Triggers model learning when new data is available, and policy learning
    whe new model is available
'''
import argparse
import atexit
import numpy as np
import os
import rospy
import threading
import traceback
import yaml

from Queue import Queue, Empty
from ros_plant import ROSPlant

from kusanagi.base import apply_controller
from kusanagi.ghost.control import RandPolicy


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


def dummy_target(task_name, task_spec, experience, task_queue):
    task_queue.put((task_name, task_spec))
    print 'called dummy learner'
    return


if __name__ == '__main__':
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
    rospy.init_node('kusanagi_ros', disable_signals=True)

    # import yaml
    config = parse_config(args.config_path)

    # init environment with first task params
    plant_params = config['tasks'].values()[0]['plant']
    env = ROSPlant(**plant_params)

    # init task queue and list of learning threads
    tasks = Queue()
    task_state = {}
    polopt_threads = []

    # populate task queue
    for task_name in config['tasks']:
        tasks.put((task_name, config['tasks'][task_name]))
        task_state[task_name] = 'init'

    # while tasks are not done
    while not all([st == 'done' for st in task_state]):
        # get new task
        new_task_ready = False
        rospy.loginfo('Waiting for new task')
        while not new_task_ready:
            try:
                name, spec = tasks.get(timeout=1)
                new_task_ready = True
            except Empty:
                pass

        state = task_state[name]
        rospy.loginfo('Executing %s task' % (name))

        # set plant parameters for current task
        plant_params = spec['plant']
        env.init_params(**plant_params)

        # load policy
        if state == 'init':
            # collect random experience
            policy = RandPolicy(maxU=spec['policy'].maxU,
                                random_walk=spec.get('random_walk', False))
            polopt_fn = dummy_target
        else:
            # TODO load policy parameters from disk
            policy = spec['policy']
            polopt_fn = spec.get('polopt__fn',
                                 config.get('default_polopt__fn',
                                            dummy_target))

        # set task horizon
        H = int(np.ceil(spec['horizon_secs']/env.dt))

        # execute tasks and collect experience data
        experience = apply_controller(env, policy, H)

        # launch learning in a separate thread
        new_thread = threading.Thread(name=name, target=polopt_fn,
                                      args=(name, spec, experience, tasks))
        polopt_threads.append(new_thread)
        new_thread.start()
