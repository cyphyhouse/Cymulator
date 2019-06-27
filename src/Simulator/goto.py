import argparse
import _thread
from util import parse_goal_pose
import importlib
import rospy
import threading
import time


def main(num_drones, num_cars, goals, wpQueued=False):
    '''
    This function will call each model's goto method to drive models towards goal points
    :param num_drones: Number of drones to drive
    :param num_cars: Number of cars to drive
    :param goals: Goal points the models are driving towards
    :return: Nothing
    '''
    loc = {
        'drone': parse_goal_pose(num_drones, goals[:num_drones], 'drone'),
        'car': parse_goal_pose(num_cars, goals[num_drones:num_drones+num_cars], 'car')
    }

    try:
        droneModule = importlib.import_module("Drone.goto")
        carModule = importlib.import_module("F1tenth.goto")
    except AttributeError:
        print("Import goto function failed!")
        exit(0)

    droneThread = None
    carThread = None
    if num_drones != 0:
        droneThread = threading.Thread(target=droneModule.GoTo, args=(num_drones, loc['drone'], wpQueued))
        droneThread.start()
    if num_cars != 0:
        carThread = threading.Thread(target=carModule.GoTo, args=(num_cars, loc['car']))
        carThread.start()
    rospy.loginfo("Models start goto method")

    # TODO: Debug to stop the cars after SIGINT
    if droneThread:
        droneThread.join()
    if carThread:
        droneThread.join()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-c", "--car", help="Number of cars to be driven", type=int)
    parser.add_argument("-d", "--drone", help="Number of drones to be driven", type=int)
    parser.add_argument("-w", "--waypoints", help="Will setup a waypoint queue if included", action="store_true")
    parser.add_argument("-G", "--goal", nargs="+", help="Goal locations of the models")

    args = parser.parse_args()
    num_drones = args.drone
    num_cars = args.car
    goals = args.goal
    wpQueued = args.waypoints

    if not num_drones:
        num_drones = 0
    if not num_cars:
        num_cars = 0
    if not goals:
        goals = []

    rospy.init_node('Model_GoTo', anonymous=True)
    main(num_drones, num_cars, goals, wpQueued)