#!/usr/bin/env python2
'''
    Client code for collecting data with a robot in an episodic manner.
    Triggers model learning when new data is available, and policy learning
    whe new model is available
'''
import argparse
import dill
import numpy as np
import os
import pickle
import rospy
import requests
import threading
import traceback
import yaml

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


def mc_pilco_polopt(task_name, task_spec, task_queue):
    '''
    executes one iteration of mc_pilco (model updating and policy optimization)
    '''
    # get task specific variables
    dyn = task_spec['transition_model']
    exp = task_spec['experience']
    pol = task_spec['policy']
    plant_params = task_spec['plant']
    immediate_cost = task_spec['cost']['graph']
    H = int(np.ceil(task_spec['horizon_secs']/plant_params['dt']))
    n_samples = task_spec.get('n_samples', 100)

    if task_state[task_name] != 'init':
        # train dynamics model. TODO block if training multiple tasks with
        # the same model
        if dyn.optimizer.loss_fn is None:
            with compile_lock:
                train_dynamics(
                    dyn, exp, pol.angle_dims,
                    wrap_angles=task_spec['wrap_angles'])
        else:
            train_dynamics(
                dyn, exp, pol.angle_dims,
                wrap_angles=task_spec['wrap_angles'])

        # init policy optimizer if needed
        optimizer = task_spec['optimizer']
        if optimizer.loss_fn is None:
            task_state[task_name] = 'compile_polopt'

            # get policy optimizer options
            split_H = task_spec.get('split_H', 1)
            mm_state = task_spec.get('mm_state', True)
            mm_cost = task_spec.get('mm_cost', True)
            noisy_policy_input = task_spec.get('noisy_policy_input', False)
            noisy_cost_input = task_spec.get('noisy_cost_input', False)
            truncate_gradient = task_spec.get('truncate_gradient', -1)
            learning_rate = task_spec.get('learning_rate', 1e-3)
            gradient_clip = task_spec.get('gradient_clip', 1.0)
            crn = task_spec.get('crn', 500)

            # get extra inputs, if needed
            import theano
            import theano.tensor as tt
            ex_in = OrderedDict(
                [(k, v) for k, v in immediate_cost.keywords.items()
                 if type(v) is tt.TensorVariable
                 and len(v.get_parents()) == 0])
            task_spec['extra_in'] = ex_in

            # build loss function
            outputs, inps, updts = mc_pilco.get_loss(
                pol, dyn, immediate_cost,
                n_samples=n_samples,
                mm_cost=mm_cost,
                mm_state=mm_state,
                noisy_cost_input=noisy_cost_input,
                noisy_policy_input=noisy_policy_input,
                split_H=split_H,
                truncate_gradient=truncate_gradient,
                crn=crn,
                intermediate_outs=True,
                **ex_in)
            loss = outputs[0]
            inps += ex_in.values()

            # add loss function as objective for optimizer
            with compile_lock:
                task_spec['rollout_fn'] = theano.function(inps, outputs, updates=updts)
                optimizer.set_objective(
                    loss, pol.get_params(symbolic=True), inps, updts,
                    clip=gradient_clip, learning_rate=learning_rate)
            task_spec['opt_iters'] = 0

        # train policy # TODO block if learning a multitask policy
        task_state[task_name] = 'update_polopt'
        # build inputs to optimizer
        p0 = plant_params['state0_dist']
        gamma = task_spec['discount']
        polopt_args = [p0.mean, p0.cov, H, gamma]
        extra_in = task_spec.get('extra_in', OrderedDict)
        if len(extra_in) > 0:
            polopt_args += [task_spec['cost']['params'][k] for k in extra_in]

        # update dyn and pol (resampling)
        def callback(*args, **kwargs):
            task_spec['opt_iters'] += 1
            crn = task_spec.get('crn', 500)
            if crn > 0  and task_spec['opt_iters'] % crn == 0:
                if hasattr(dyn, 'update'):
                    dyn.update(n_samples)
                if hasattr(pol, 'update'):
                    pol.update(n_samples)
        # call minimize
        callback()
        optimizer.minimize(
            *polopt_args, return_best=task_spec['return_best'],
            callback=callback)
        pol.save(output_filename='policy_%s_%d.zip'% (task_name, exp.n_episodes()))
        task_state[task_name] = 'ready'

    # check if task is done
    init_polopt_iter = task_spec.get('init_polopt_iter', 0)
    n_polopt_iters = len([p for p in exp.policy_parameters if len(p) > 0])
    if n_polopt_iters > task_spec['n_opt'] + init_polopt_iter:
        task_state[task_name] = 'done'
    # put task in the queue for execution
    task_queue.put((task_name, task_spec))

    return


def http_polopt(task_name, task_spec, task_queue):
    # TODO: Automate the harcoded url requests and responses
    url = "http://mc_pilco_server:8008/get_task_init_status/%s" % task_name

    # check if task id exists in server
    http_response = requests.get(url)
    rospy.loginfo(http_response.text)

    # if task_name doesn't exists then upload the task_spec for task_name
    if http_response.text == "get_task_init_status/%s: NOT FOUND" % task_name:
        url = "http://mc_pilco_server:8008/init_task/%s" % task_name
        tspec_pkl = pickle.dumps(task_spec, 2)
        http_response = requests.post(
            url, files={'tspec_file': ('task_spec.pkl', tspec_pkl)})
        rospy.loginfo(http_response.text)
        # TODO: Error Handling

    # TODO: Error handling if the task_spec upload fails
    # send latest experience for task_name
    url = "http://mc_pilco_server:8008/optimize/%s" % task_name
    exp_pkl = pickle.dumps(task_spec['experience'], 2)
    pol_params_pkl = pickle.dumps(
        task_spec['policy'].get_params(symbolic=False), 2)

    http_response = requests.post(
        url,
        files={
            'exp_file': ('experiance.pkl', exp_pkl),
            'pol_params_file': ('policy_params.pkl', pol_params_pkl)
        }
    )

    pol_params = pickle.loads(http_response.text)
    task_spec['policy'].set_params(pol_params)

    task_queue.put((task_name, task_spec))


if __name__ == '__main__':
    np.set_printoptions(linewidth=200, precision=3)
    rospy.init_node('kusanagi_ros', disable_signals=True)

    parser = argparse.ArgumentParser(
        'rosrun robot_learning task_client.py')
    parser.add_argument(
        'config_path', metavar='FILE',
        help='A YAML file containing the configuration for the learning task.',
        type=str)
    parser.add_argument(
        '-e', '--load_experience',
        help="load past experience if available",
        action="store_true")
    # args = parser.parse_args()
    args = parser.parse_args(rospy.myargv()[1:])
    load_experience = args.load_experience

    # import yaml
    config = parse_config(args.config_path)

    # init output dir
    output_directory = config['output_directory']
    utils.set_output_dir(output_directory)
    try:
        os.makedirs(output_directory)
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
        pol(np.zeros(pol.D))
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
                # exp.truncate(spec['initial_random_trials'])
                spec['initial_random_trials'] -= exp.n_episodes()
                # in case we are loading policy parameters, increase the n_opt counter
                n_polopt_iters = len([p for p in exp.policy_parameters if len(p) > 0])
                spec['init_polopt_iter'] = n_polopt_iters
                print n_polopt_iters
                task_state[task_name] = 'ready'
        spec['experience'] = exp
        # trigger policy init (for kusanagi only)

        # Optimize policy on the loaded experience
        if exp.n_episodes() > 0:
            polopt_fn = spec.get('polopt_fn',
                        config.get('default_polopt_fn',
                                mc_pilco_polopt))
            polopt_fn(task_name, spec, tasks)
        else:
            tasks.put((task_name, spec))
        with open(
            os.path.join(
                output_directory, '%s_spec.dill') % (task_name), 'wb') as f:
            dill.dump(spec, f)

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

        msg_ = '==== Executing %s task [iteration %d] ====' % (
            name, exp.n_episodes()+1-n_rnd)
        rospy.loginfo(msg_)
        #utils.print_with_stamp(msg_)

        # set plant parameters for current task
        plant_params = spec['plant']
        env.init_params(**plant_params)

        # load policy
        if task_state[name] == 'init' and spec['initial_random_trials'] > 0:
            # collect random experience
            pol = RandPolicy(maxU=spec['policy'].maxU,
                             minU=spec['policy'].minU,
                             random_walk=spec.get('random_walk', False))
            spec['initial_random_trials'] -= 1
            if spec['initial_random_trials'] < 1:
                task_state[name] = 'ready'
        else:
            # TODO load policy parameters from disk
            pol = spec['policy']

        polopt_fn = spec.get('polopt_fn',
                             config.get('default_polopt_fn',
                                        mc_pilco_polopt))

        # set task horizon
        H = int(np.ceil(spec['horizon_secs']/env.dt))

        # execute tasks and collect experience data
        preprocess = None
        if hasattr(pol, 'angle_dims'):
            preprocess = partial(
                preprocess_angles, angle_dims=pol.angle_dims)
        experience = apply_controller(env, pol, H, preprocess)
        # print experience[0]
        # append new experience to dataset
        states, actions, costs, infos = experience
        ts = [info.get('t', None) for info in infos]
        pol_params = (pol.get_params(symbolic=False)
                      if hasattr(pol, 'get_params') else [])

        exp.append_episode(
            states, actions, costs, infos, pol_params, ts)

        exp.save()
        if task_state[name] == 'init':
            # save random experience for later
            fname = exp.filename
            exp.save(base_path, filename)
            # restore previous filename
            exp.filename = fname

        if task_state[name] == 'done':
            rospy.loginfo(
                'Finished %s task [iteration %d]' % (
                    name, exp.n_episodes()-n_rnd))
            break # TODO don't break when running multiple tasks

        spec['experience'] = exp

        # launch learning in a separate thread
        #new_thread = threading.Thread(name=name, target=polopt_fn,
        #                              args=(name, spec, tasks))
        #polopt_threads.append(new_thread)
        #new_thread.start()
        polopt_fn(name, spec, tasks)
        # http_polopt(name, spec, tasks)
