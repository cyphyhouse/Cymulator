#!/usr/bin/env python3
"""
    Move according to a log file
    Doesn't involve physics

"""
import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
import argparse

Mode = 0


def init(num, logfile):
    '''
    This function will call move function according to the mode specified
    :param argv: A list of arguments - [mode, number, positions/logfile name]
    :return: Nothing
    '''
    drones = []

    for i in range(num):
        drone_msg = ModelState()
        drone_msg.model_name = 'drone'+str(i+1)
        drones.append(drone_msg)

    if logfile == "":  # reset drones position
        for i in range(num):
            move(drones[i], (2*i, 0))
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
    state_msg.pose.position.z = 0
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
    parser.add_argument("-d", "--drone", help="Number of drones to be moved", type=int)
    parser.add_argument("-L", "--log", help="Name of logfile; if leave empty, it'll reset models' location", type=str)
    args = parser.parse_args()
    num_drones = args.drone
    logfile = args.log
    try:
        init(num_drones, logfile)
    except rospy.ROSInterruptException:
        rospy.loginfo("User pressed  Ctrl-C, quit!")