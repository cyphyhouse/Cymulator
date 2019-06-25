import sys
import os
from util import parse_init_pose, sim_launch
import argparse


def main():
    '''
    This function configure and launch Gazebo-ROS simulator
    :param argv: user command line input in format - MODEL_TYPE, NUMBER_OF_MODEL
    :return: Nothing; the program exits when user press Ctrl-C or shutdown Gazebo
    '''
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
    main()
