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



class ServiceExit(Exception):
    """
    Custom exception which is used to trigger the clean exit
    of all running threads and the main program.
    """
    pass
 
 
def service_shutdown(signum, frame):
    print('Caught signal %d' % signum)
    raise ServiceExit





# TODO: refactor the code to rename this module to simulator object...
def main(num_drones, num_cars, goals, wpQueued=False):
    '''
    This function will call each model's goto method to drive models towards goal points
    :param num_drones: Number of drones to drive
    :param num_cars: Number of cars to drive
    :param goals: Goal points the models are driving towards
    :return: Nothing
    '''

    # Parse the input goals of models; if user does not enter any locations, give random locations
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

    # Register the signal handlers
    signal.signal(signal.SIGTERM, service_shutdown)
    signal.signal(signal.SIGINT, service_shutdown)
    print('Starting main program')

    # Start separate threads for different models
    droneThread = None
    carThread = None
    
    if num_drones != 0:
        droneList = [ droneModule.Drone(i + 1, loc['drone'][i], wpQueued) for i in range( num_drones) ]
        droneThreadList = [ threading.Thread(target=drone.controller) for drone in droneList ]
        
        for droneThread in droneThreadList:
            droneThread.start()

        # droneThread = threading.Thread(target=droneModule.GoTo, args=(num_drones, loc['drone'], wpQueued))
        # droneThread.start()
    if num_cars != 0:
        carList = [ carModule.Car(i + 1, loc['car'][i]) for i in range( num_cars) ]
        carThreadList = [ threading.Thread(target=car.goto) for car in carList ]
        for carThread in carThreadList:
            carThread.start()

    rospy.loginfo("Models start goto method")

    while True:
        try:
            time.sleep(0.5)

        except ServiceExit:
            # Terminate the running threads.
            # Set the shutdown flag on each thread to trigger a clean shutdown of each thread.
            if(not carList == None):
                for car in carList:
                    car.shutdown_flag = 1

            if droneThread:
                for droneThread in droneThreadList:
                    droneThread.join()
            if carThreadList:
                for carThread in carThreadList:
                    carThread.join()
            break
 
    print('Exiting main program')

   


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