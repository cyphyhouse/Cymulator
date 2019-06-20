#!/usr/bin/env python3
"""
    Move according to a log file
    Doesn't involve physics

"""
import rospy
import rospkg
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from logParser import parse
import sys

Mode = 0

def main(argv):
    if len(argv) < 3:
        print("You must enter the mode and the number of drones! ")
        exit(0)
    mode = int(argv[1])
    num = int(argv[2])
    rospy.init_node('Move')
    drones = []

    for i in range(num):
        drone_msg = ModelState()
        drone_msg.model_name = 'drone'+str(i+1)
        drones.append(drone_msg)

    if mode == 0:  # reset drones position
        for i in range(num):
            move(drones[i], (2*i, 0))
    elif mode == 1:  # Set drones to particular positions
        assert (len(argv)-3 == num), "Must provide exact the same number of positions to the number of drones"

        for i in range(3, len(argv)):
            pose = tuple(int(j) for j in argv[i].strip('()').split(','))
            move(drones[i-3], pose)
    # elif mode == 2 :  # Reserved for later log use
    #     path = parse()
    #     for point1, point2 in path:
    #         move(robot1_msg, point1)
    #         move(robot2_msg, point2)


def move(state_msg, pose):
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
    try:
        main(sys.argv)
    except rospy.ROSInterruptException:
        rospy.loginfo("User pressed  Ctrl-C, quit!")