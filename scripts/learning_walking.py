#!/usr/bin/env python2

import rospy
from robot_learning.ros_plant import ROSPlant
from robot_learning.srv import T2VInfo
import numpy as np


def tripod_gait(state, cmd, dt, period=3.0, offset=4*np.pi/12,
                sweep_angle=np.pi/2, duty_factor=0.75):
    leg_angles = cmd[:6]
    slow_legs = (np.abs(
                    (leg_angles - offset + np.pi) % (2*np.pi)
                    - np.pi) < sweep_angle/2.0)
    fast_legs = np.logical_not(slow_legs)
    slow_legs = np.where(slow_legs)[0]
    fast_legs = np.where(fast_legs)[0]

    slow_speed = -(2*np.pi)/(period*duty_factor)
    fast_speed = -(2*np.pi)/(period*(1.0-duty_factor))
    cmd[6+slow_legs] = slow_speed
    cmd[slow_legs] = (cmd[slow_legs] +
                      dt*slow_speed) % (2*np.pi)

    cmd[6+fast_legs] = fast_speed
    cmd[fast_legs] = (cmd[fast_legs] +
                      dt*fast_speed) % (2*np.pi)
    return cmd


if __name__ == '__main__':
    rospy.init_node('learning_to_walk')

    env = ROSPlant(dt=rospy.get_param('~dt', 0.05))
    state = env.reset()
    t = rospy.get_time()
    command_dims_service = rospy.ServiceProxy('/rl/command_dims', T2VInfo)
    command_dims = command_dims_service().value

    cmd = np.zeros(command_dims)
    cmd[:6] = 0.785
    cmd[1] += np.pi
    cmd[3] += np.pi
    cmd[5] += np.pi
    prev_t = rospy.get_time()
    while not rospy.is_shutdown():
        t = rospy.get_time()
        dt = (t - prev_t)
        prev_t = t

        cmd = tripod_gait(state, cmd, dt)

        state, reward, info, done = env.step(cmd)
