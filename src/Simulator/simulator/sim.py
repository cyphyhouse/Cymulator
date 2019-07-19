import sys, os, argparse, multiprocessing
from util import parse_init_pose, sim_launch
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


def main1(num_drones, num_cars):
    '''
    This function will call each model's goto method to drive models towards goal points
    :param num_drones: Number of drones to drive
    :param num_cars: Number of cars to drive
    :param goals: Goal points the models are driving towards
    :return: Nothing
    '''
    rospy.init_node('Model_GoTo', anonymous=True)
    
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



def main(num_drones, num_cars, init_loc):
    '''
    This function configure and launch Gazebo-ROS simulator
    :param num_drones: Number of drones to generate
    :param num_cars: Number of cars to generate
    :param init_loc: Initail locations for the models
    :return: Nothing, simulator will shutdown if user press Ctrl+C
    '''

    loc = {
        'drone': parse_init_pose(num_drones, init_loc[:num_drones]),
        'car': parse_init_pose(num_cars, init_loc[num_drones:num_drones+num_cars])
    }

    models = {'drone': num_drones, 'car': num_cars}

    print("Generate Models")
    proc = sim_launch(models, loc)

    # Infinite loop like ros spin
    while True:
        try:
            pass
        except KeyboardInterrupt:
            # proc.kill()
            os.system("killall -2 rosmaster")
            os.system("killall -2 gzserver")
            print("User pressed Ctrl-C, exit! ")
            sys.exit(0)


if __name__ == '__main__':
    # python3 sim.py NUM_OF_DRONES NUM_OF_CARS INIT_POSITIONS
    parser = argparse.ArgumentParser()
    parser.add_argument("-c", "--car", help="Number of cars to be generated", type=int)
    parser.add_argument("-d", "--drone", help="Number of drones to be generated", type=int)
    parser.add_argument("-I", "--initial", nargs="+", help="Initial locations of the models")

    args = parser.parse_args()
    num_drones = args.drone
    num_cars = args.car
    init_loc = args.initial
    if not num_drones:
        num_drones = 0
    if not num_cars:
        num_cars = 0
    if not init_loc:
        init_loc = []

    gazebo = multiprocessing.Process(target=main, args=(num_drones, num_cars, init_loc) )
    

    model = multiprocessing.Process(target=main1, args=(num_drones, num_cars) )

    gazebo.start()
    time.sleep( 10 + 1.5 * (num_drones +  num_cars) )
    model.start()
    print("********************************** initilization finished **************************************")

