#!/usr/bin/env python3

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
import argparse
import sys


def parseLog(filename):
    f = open(filename, "r")
    lines = f.readlines()
    path = []
    for x in lines:
        data = x.split(',')
        pos = (float(data[1]), float(data[2]), float(data[3]))
        quat = (float(data[4]), float(data[5]), float(data[6]), float(data[5]))
        path.append((pos, quat))

    f.close()
    return path


def init(num, logfile, random_pos=False):
    cars = []

    for i in range(num):
        car_msg = ModelState()
        car_msg.model_name = 'car'+str(i+1)
        cars.append(car_msg)

    if logfile == "":  # reset drones position
        if not random_pos:
            for i in range(num):
                move(cars[i], (0, 2 * i + 1, 0.18))
        else:
            sys.path.append('..')
            from util import parse_init_pose
            poses = parse_init_pose(num, [])
            for i in range(num):
                move(cars[i], poses[i])
    # TODO: Need an agreement on the format logfile before set up this branch
    else:
        path = parseLog(logfile)
        for pos, quat in path:
            move(cars[0], pos, quat)


def move(state_msg, pose, quat=[0, 0, 0, 0]):
    '''
        This function will directly set drone's state (including poses and orientations)
        :param state_msg: current set model's state message
        :param pose: the pose to be set
        :return: Nothing
    '''
    rospy.loginfo("Currently on (%f, %f)", pose[0], pose[1])
    state_msg.pose.position.x = pose[0]
    state_msg.pose.position.y = pose[1]
    state_msg.pose.position.z = 0
    state_msg.pose.orientation.x = quat[0]
    state_msg.pose.orientation.y = quat[1]
    state_msg.pose.orientation.z = quat[2]
    state_msg.pose.orientation.w = quat[3]

    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        resp = set_state(state_msg)
    except rospy.ServiceException as e:
        rospy.loginfo("Service call failed with %s", e)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-c", "--car", help="Number of cars to be moved", type=int)
    parser.add_argument("-r", "--random", help="Set drones on random states if included", action="store_true")
    parser.add_argument("-L", "--log", help="Name of logfile; if leave empty, it'll reset models' location", type=str)
    args = parser.parse_args()
    num_cars = args.car
    logfile = args.log
    random_pos = args.random
    try:
        init(num_cars, logfile, random_pos)
    except rospy.ROSInterruptException:
        rospy.loginfo("User pressed  Ctrl-C, quit!")