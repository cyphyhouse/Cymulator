import argparse
import importlib
import rospy
import threading


def main(num_drones, num_cars, logfile):
    try:
        droneModule = importlib.import_module("Drone.move")
        carModule = importlib.import_module("F1tenth.move")
    except AttributeError:
        print("Import goto function failed!")
        exit(0)

    if num_drones != 0:
        droneThread = threading.Thread(target=droneModule.init, args=(num_drones, logfile))
        droneThread.start()
    if num_cars != 0:
        carThread = threading.Thread(target=carModule.init, args=(num_cars, logfile))
        carThread.start()
    rospy.loginfo("Models start move method")


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-c", "--car", help="Number of cars to be moved", type=int)
    parser.add_argument("-d", "--drone", help="Number of drones to be moved", type=int)
    parser.add_argument("-L", "--log", help="Name of logfile; if leave empty, it'll reset models' location", type=str)

    args = parser.parse_args()
    num_drones = args.drone
    num_cars = args.car
    logfile = args.log

    if not num_drones:
        num_drones = 0
    if not num_cars:
        num_cars = 0
    if not logfile:
        logfile = ""
    rospy.init_node("Model_Move", anonymous=True)
    main(num_drones, num_cars, logfile)