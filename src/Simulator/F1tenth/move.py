#!/usr/bin/env python3

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
import argparse


def init(num, logfile):
    cars = []

    for i in range(num):
        car_msg = ModelState()
        car_msg.model_name = 'car'+str(i+1)
        cars.append(car_msg)

    if logfile == "":  # reset drones position
        for i in range(num):
            move(cars[i], (2 * i, 0))
    # TODO: Need an agreement on the format logfile before set up this branch
    # else:
    #     path = parse()
    #     for point1, point2 in path:
    #         move(robot1_msg, point1)
    #         move(robot2_msg, point2)

def move(state_msg, pose):
    '''
        This function will directly set drone's state (including poses and orientations)
        :param state_msg: current set model's state message
        :param pose: the pose to be set
        :return: Nothing
    '''
    rospy.loginfo("Currently on (%f, %f)", pose[0], pose[1])
    state_msg.pose.position.x = pose[0]
    state_msg.pose.position.y = pose[1]
    state_msg.pose.position.z = 0.0
    state_msg.pose.orientation.x = 0
    state_msg.pose.orientation.y = 0
    state_msg.pose.orientation.z = 0
    state_msg.pose.orientation.w = 0

    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        resp = set_state(state_msg)
    except rospy.ServiceException as e:
        rospy.loginfo("Service call failed with %s", e)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-c", "--car", help="Number of cars to be moved", type=int)
    parser.add_argument("-L", "--log", help="Name of logfile; if leave empty, it'll reset models' location", type=str)
    args = parser.parse_args()
    num_cars = args.car
    logfile = args.log
    try:
        init(num_cars, logfile)
    except:
        rospy.loginfo("User pressed  Ctrl-C, quit!")