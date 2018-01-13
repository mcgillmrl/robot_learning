#!/usr/bin/env python2
import socket
from kusanagi.ghost.algorithms import mc_pilco
from time import sleep

def model_learning_server():
    '''
    launches a subprocess that will listen on a socket for new
    '''
    # start tcp socket  server
    sock = socket.socket()


def launch_policy_learning_process():
    pass


def mc_pilco_opt(task_name, task_spec, experience, task_queue):
    # update experience dataset
    print experiment_utils
    task_queue.put((task_name, task_spec))
    print 'called mc_pilco learner'
    sleep(10)


def polopt(task_spec):
    loss_kwargs = kwargs.pop('loss_kwargs', {})
    polopt_kwargs = kwargs.pop('polopt_kwargs', {})
    # build loss function
    loss, inps, updts = mc_pilco.get_loss(
        pol, dyn, cost, **loss_kwargs)

    inps += kwargs.pop('extra_inps', [])

    # set objective of policy optimizer
    polopt.set_objective(loss, pol.get_params(symbolic=True),
                         inps, updts, **polopt_kwargs)
