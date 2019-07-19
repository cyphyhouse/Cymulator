import argparse
import _thread
from util import parse_goal_pose
import importlib
import rospy
import threading
import time
import os
import multiprocessing
import signal


def main(num_drones, num_cars):
    '''
    This function will call each model's goto method to drive models towards goal points
    :param num_drones: Number of drones to drive
    :param num_cars: Number of cars to drive
    :param goals: Goal points the models are driving towards
    :return: Nothing
    '''

    try:
        droneModule = importlib.import_module("Drone.goto")
        carModule = importlib.import_module("F1tenth.goto")
    except AttributeError:
        print("Import goto function failed!")
        exit(0)

    # Start separate threads for different models
    droneThread = None
    carThread = None
    
    if num_drones != 0:
        droneList = [ droneModule.Drone(i + 1) for i in range( num_drones) ]
        droneThreadList = [ threading.Thread(target=drone.controller) for drone in droneList ]
        for droneThread in droneThreadList:
            droneThread.start()

    if num_cars != 0:
        carList = [ carModule.Car(i + 1) for i in range( num_cars) ]
        carThreadList = [ threading.Thread(target=car.controller) for car in carList ]
        for carThread in carThreadList:
            carThread.start()

    rospy.loginfo("Models start goto method")

    while True:
        time.sleep(0.5)

    print('Exiting main program')

   


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-c", "--car", help="Number of cars to be driven", type=int)
    parser.add_argument("-d", "--drone", help="Number of drones to be driven", type=int)

    args = parser.parse_args()
    num_drones = args.drone
    num_cars = args.car

    if not num_drones:
        num_drones = 0
    if not num_cars:
        num_cars = 0

    rospy.init_node('Model_GoTo', anonymous=True)
    main(num_drones, num_cars)