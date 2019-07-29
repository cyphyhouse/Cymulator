import argparse
import os
import sys

from cymulator import util


def main(num_drones, num_cars, init_loc):
    '''
    This function configure and launch Gazebo-ROS simulator
    :param num_drones: Number of drones to generate
    :param num_cars: Number of cars to generate
    :param init_loc: Initail locations for the models
    :return: Nothing, simulator will shutdown if user press Ctrl+C
    '''

    loc = {
        'drone': util.parse_init_pose(num_drones, init_loc[:num_drones]),
        'car': util.parse_init_pose(num_cars, init_loc[num_drones:num_drones+num_cars])
    }

    models = {'drone': num_drones, 'car': num_cars}

    print("Generate Models")
    proc = util.launch(models, loc)

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
    # python3 -m cymulator.sim NUM_OF_DRONES NUM_OF_CARS INIT_POSITIONS
    parser = argparse.ArgumentParser()
    parser.add_argument("-c", "--car", help="Number of cars to be generated", type=int)
    parser.add_argument("-d", "--drone", help="Number of drones to be generated", type=int)
    # TODO parse to coordinate like type directly
    # E.g., https://stackoverflow.com/questions/9978880/python-argument-parser-list-of-list-or-tuple-of-tuples
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

    print(init_loc)
    main(num_drones, num_cars, init_loc)
