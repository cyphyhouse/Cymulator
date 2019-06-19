#!/usr/bin/env python3

import rospy
import rospkg
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from logParser import parse
from spawn import Spawn
from math import sqrt

Mode = 1


def main():
    rospy.init_node('Move')

    robot1_msg = ModelState()
    robot1_msg.model_name = 'robot1'

    robot2_msg = ModelState()
    robot2_msg.model_name = 'robot2'

    track = Spawn()

    if Mode == 0:
        move(robot1_msg, (0,0))
        move(robot2_msg, (0,2))
    elif Mode == 1:
        path = parse()
        for i in range(len(path)):
            if i % 80 == 0:
                next_idx = i+80
            else:
                next_idx = 0
            if next_idx != 0 and next_idx < len(path):
                rospy.loginfo("next idx is %d", next_idx)
                curSpawn = path[next_idx][0]
            point1, point2 = path[i]
            preSpawn = point1
            move(robot1_msg, point1)
            move(robot2_msg, point2)

            difference = ((curSpawn[0]-preSpawn[0])*(curSpawn[0]-preSpawn[0]) + (curSpawn[1]-preSpawn[1])*(curSpawn[1]-preSpawn[1]))
            if next_idx != 0 and difference > 0.01:
                rospy.loginfo("curSpawn at %d, (%f,%f), Difference is %f", next_idx, curSpawn[0], curSpawn[1],
                              difference)
                track.create_point(curSpawn[0], curSpawn[1], 0, next_idx)
                track.paths[next_idx] = True

            if i in track.paths:
                rospy.loginfo("delete point at %d", i)
                track.delete_point(i)


def move(state_msg, pose):
    # rospy.loginfo("Currently on (%f, %f)", pose[0], pose[1])
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
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("User pressed  Ctrl-C, quit!")