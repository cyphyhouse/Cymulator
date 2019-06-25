import argparse
import _thread
from util import parse_goal_pose
import importlib
import rospy
import threading


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-c", "--car", help="Number of cars to be driven", type=int)
    parser.add_argument("-d", "--drone", help="Number of drones to be driven", type=int)
    parser.add_argument("-G", "--goal", nargs="+", help="Goal locations of the models")

    args = parser.parse_args()
    num_drones = args.drone
    num_cars = args.car
    goals = args.goal

    if not num_drones:
        num_drones = 0
    if not num_cars:
        num_cars = 0
    if not goals:
        goals = []

    loc = {
        'drone': parse_goal_pose(num_drones, goals[:num_drones], 'drone'),
        'car': parse_goal_pose(num_cars, goals[num_drones:num_drones+num_cars], 'car')
    }

    rospy.init_node('GoTo_Test', anonymous=True)
    try:
        droneModule = importlib.import_module("Drone.goto")
        carModule = importlib.import_module("F1tenth.goto")
    except AttributeError:
        print("Import goto function failed!")

    # module.GoTo(num_drones, loc['drone'])
    if num_drones != 0:
        droneThread = threading.Thread(target=droneModule.GoTo, args=(num_drones, loc['drone']))
        droneThread.start()
    if num_cars != 0:
        carThread = threading.Thread(target=carModule.GoTo, args=(num_cars, loc['car']))
        carThread.start()
    rospy.loginfo("Models start goto method")



    rospy.loginfo("----TEST----")


if __name__ == '__main__':
    main()