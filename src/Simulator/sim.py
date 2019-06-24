import sys
import os
from util import parse_init_pose, sim_launch


def main(argv):
    '''
    This function configure and launch Gazebo-ROS simulator
    :param argv: user command line input in format - MODEL_TYPE, NUMBER_OF_MODEL
    :return: Nothing; the program exits when user press Ctrl-C or shutdown Gazebo
    '''
    if len(argv) < 3:
        print("Please enter arguments in form of: ")
        print("     python3 sim.py NUM_OF_DRONES NUM_OF_CARS INIT_POSITIONS")
        exit(0)

    num_drones, num_cars = argv[1:3]
    num_drones = int(num_drones)
    num_cars = int(num_cars)
    loc = {
        'drone': parse_init_pose(num_drones, argv[3:3+num_drones]),
        'car': parse_init_pose(num_cars, argv[3+num_drones:3+num_drones+num_cars])
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
            os.system("killall -9 rosmaster")
            os.system("killall -9 gzserver")
            print("User pressed Ctrl-C, exit! ")
            sys.exit(0)


if __name__ == '__main__':
    # python3 sim.py NUM_OF_DRONES NUM_OF_CARS INIT_POSITIONS
    main(sys.argv)
