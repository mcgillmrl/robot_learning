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
import yaml

from collections import OrderedDict
from functools import partial
from Queue import Queue, Empty
from ros_plant import ROSPlant

from kusanagi.base import (apply_controller, train_dynamics,
                           preprocess_angles, ExperienceDataset)
from kusanagi.ghost.algorithms import mc_pilco
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


def mc_pilco_polopt(task_name, task_spec, experience, task_queue):
    '''
    executes one iteration of mc_pilco (model updating and policy optimization)
    '''
    # get task specific variables
    n_samples = task_spec['n_samples']
    dyn = task_spec['transition_model']
    exp = task_spec.get('experience_data', ExperienceDataset())
    pol = task_spec['policy']
    plant_params = task_spec['plant']
    immediate_cost = task_spec['cost']['graph']

    # append new experrience to dataset
    task_state[task_name] = 'update_dyn'
    states, actions, costs, infos = experience
    ts = [info.get('t', None) for info in infos]
    exp.append_episode(states, actions, costs, infos,
                       pol.get_params(symbolic=False), ts)
    task_spec['experience_data'] = exp

    # train dynamics model. TODO block if training multiple tasks with
    # the same model
    train_dynamics(dyn, exp, pol.angle_dims,
                   wrap_angles=task_spec['wrap_angles'])

    # init policy optimizer if needed
    optimizer = task_spec['optimizer']
    if optimizer.loss_fn is None:
        task_state[task_name] = 'compile_polopt'
        import theano.tensor as tt
        ex_in = OrderedDict([(k, v) for k, v in immediate_cost.keywords.items()
                             if type(v) is tt.TensorVariable
                             and len(v.get_parents()) == 0])
        task_spec['extra_in'] = ex_in
        loss, inps, updts = mc_pilco.get_loss(
            pol, dyn, immediate_cost, n_samples=n_samples, **ex_in)
        inps += ex_in.values()
        optimizer.set_objective(
            loss, pol.get_params(symbolic=True), inps, updts)

    # train policy # TODO block if learning a multitask policy
    task_state[task_name] = 'update_polopt'
    # build inputs to optimizer
    p0 = plant_params['state0_dist']
    H = int(np.ceil(task_spec['horizon_secs']/plant_params['dt']))
    gamma = task_spec['discount']
    polopt_args = [p0.mean, p0.cov, H, gamma]
    extra_in = task_spec.get('extra_in', OrderedDict)
    if len(extra_in) > 0:
        polopt_args += [task_spec['cost']['params'][k] for k in extra_in]
    # update dyn and pol (resampling)
    if hasattr(dyn, 'update'):
        dyn.update(n_samples)
    if hasattr(pol, 'update'):
        pol.update(n_samples)
    # call minimize
    optimizer.minimize(*polopt_args,
                       return_best=task_spec['return_best'])

    # put task in the queue for execution
    task_state[task_name] = 'ready'
    task_queue.put((task_name, task_spec))

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
        spec = config['tasks'][task_name]
        policy = spec['policy']
        task_state[task_name] = 'init'
        # trigger policy init (for kusanagi only)
        policy.evaluate(np.zeros(policy.D))
        tasks.put((task_name, spec))

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
            polopt_fn = mc_pilco_polopt
        else:
            # TODO load policy parameters from disk
            policy = spec['policy']
            polopt_fn = spec.get('polopt_fn',
                                 config.get('default_polopt_fn',
                                            mc_pilco_polopt))

        # set task horizon
        H = int(np.ceil(spec['horizon_secs']/env.dt))

        # execute tasks and collect experience data
        preprocess = None
        if hasattr(policy, 'angle_dims'):
            preprocess = partial(
                preprocess_angles, angle_dims=policy.angle_dims)
        experience = apply_controller(env, policy, H, preprocess)

        # launch learning in a separate thread
        new_thread = threading.Thread(name=name, target=polopt_fn,
                                      args=(name, spec, experience, tasks))
        polopt_threads.append(new_thread)
        new_thread.start()
