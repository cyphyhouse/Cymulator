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
        print("Please enter arguments in form of: python3 demo.py MODEL_TYPE NUMBER_OF_MODEL INIT_POSITIONS")
        exit(0)

    model, num = argv[1:3]
    num = int(num)
    init_poses = parse_init_pose(num, argv[3:])

    if model == 'drone':
        print("Generate drones")
        proc = sim_launch(num, './Drone', init_poses)

    elif model == 'car' or model == 'f1tenth':
        print("Generate cars")
        proc = sim_launch(num, './F1tenth', init_poses)

    # Infinite loop like ros spin
    while True:
        try:
            # print("PASS")
            pass
        except KeyboardInterrupt:
            # proc.kill()
            os.system("killall -9 rosmaster")
            os.system("killall -9 gzserver")
            print("User pressed Ctrl-C, exit! ")
            sys.exit(0)


if __name__ == '__main__':
    # python3 demo.py MODEL_TYPE NUMBER_OF_MODEL INIT_POSITIONS*
    main(sys.argv)
