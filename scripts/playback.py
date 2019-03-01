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
import rospy
import sys

from kusanagi.base import ExperienceDataset
from kusanagi.shell.cost import build_loss_func
from kusanagi.shell.experiment_utils import evaluate_policy
from kusanagi import utils
from robot_learning.ros_plant import ROSPlant


def check_files_suffix(files, suffix):
    return [fname for fname in files if fname.find(suffix) > 0]


if __name__ == '__main__':
    np.set_printoptions(linewidth=200, precision=3)
    rospy.init_node('kusanagi_ros', disable_signals=True)

    parser = argparse.ArgumentParser('rosrun robot_learning task_client.py')
    parser.add_argument(
        'results_path',
        metavar='DIR',
        help='The location of the results .dill and zip files',
        type=str)
    parser.add_argument(
        '-n',
        '--n_evals',
        metavar='N',
        default=1,
        help='Run each policy N times per episode',
        type=int)
    parser.add_argument(
        '-i',
        '--iter',
        metavar='I',
        default=0,
        help='Start from policy obtained at iteration i',
        type=int)
    parser.add_argument(
        '-w',
        '--cost_width',
        metavar='w',
        help='cost_width for distance based cost',
        type=float)

    # args = parser.parse_args()
    args = parser.parse_args(rospy.myargv()[1:])
    load_experience = True

    # init output dir
    utils.set_output_dir(args.results_path)
    utils.print_with_stamp('Results will be saved in [%s]' % args.results_path)

    # load config
    files = os.listdir(args.results_path)
    spec_paths = check_files_suffix(files, '_spec.dill')
    if len(spec_paths) == 0:
        utils.print_with_stamp("No *_spec.dill file found. Quitting...")
        sys.exit(-1)
    spec_path = os.path.join(args.results_path, spec_paths[0])
    print spec_path, spec_paths
    f = open(spec_path)  # TODO what if we have more than one dill file
    config = dill.load(f)

    #  load experience dataset
    exp_paths = check_files_suffix(files, '_dataset.zip')
    if len(spec_paths) == 0:
        utils.print_with_stamp("No *_dataset.zip file found. Quitting...")
        sys.exit(-1)
    exp = ExperienceDataset(filename=exp_paths[0].split('.')[0])

    # init environment with task params
    plant_params = config['plant']
    print 'COST WIDTH', config['cost']['graph'].func.keywords['cw']
    if args.cost_width is not None:
        # replace cost function with new one
        cost_params = config['cost']['params']
        config['cost']['graph'].func.keywords['cw'] = args.cost_width
        loss = config['cost']['graph']
        plant_params['loss_func'] = build_loss_func(loss,
                                                    **config['cost']['params'])

    env = ROSPlant(**plant_params)

    # get dynamics model and policy
    angle_dims = config['angle_dims']
    dyn = config['transition_model']
    pol = config['policy']

    H = int(np.ceil(config['horizon_secs'] / config['plant']['dt']))
    config['min_steps'] = H
    #episodes = [i+5 for i in [0, 4, 9, 12, 26, 38, 39]]
    #episodes = [i+5 for i in [ 0 , 3 , 9, 15, 29, 36, 37]]
    #episodes = [i+5 for i in [ 0,  4,  9, 18, 26, 39, 46]]
    #episodes = [i+5 for i in [0,  4, 8, 18, 19, 37, 43]]
    #episodes = [i+5 for i in [ 0,  4,  8, 19, 24, 33, 49]]
    #episodes = [i+5 for i in [ 0,  2,  9, 19, 29, 36, 43]]
    #episodes = [i+5 for i in [ 0,  3,  6, 19, 29, 38, 44]]
    #episodes = [i+5 for i in [ 0,  4,  9, 17, 26, 38, 39]]
    episodes = range(args.iter, exp.n_episodes())
    print episodes

    results = evaluate_policy(
        env, pol, exp, config, filter_episodes=episodes, n_tests=args.n_evals)
    with open(
            os.path.join(args.results_path, 'results_%d.dill') %
        (args.n_evals), 'wb') as f:
        dill.dump(results, f)
