import sys
import rospy
import subprocess
import time
import os


def main(argv):

    '''
    This function prompt instructions to ask user to input to configure and launch Gazebo simulator
    :param argv: user command line input
    :return: Nothing; the program exits when user press Ctrl-C or shutdown Gazebo
    '''

    if len(argv) < 3:
        print("Must choose a model type and specify the number of models")
        exit(0)
    num = int(argv[2])
    if argv[1] == 'drone' or 'Drone':
        ans = input("What would you like the drones to do?\n"
                    "0 - Nothing, just open simulator;\n"
                    "1 - Go to specific positions with physics;\n"
                    "2 - Set drones' states manually or according to logfile\n")
        ans = int(ans)

        if ans == 1:
            print("Since you choose 1, please enter the goals of drones or the drone will go to default goals")
        if ans == 2:
            print("Since you choose 2, please choose the mode")
            move_mode = input("0 - reset drone positions\n"
                              "1 - enter positions you would like\n"
                              "2 - set according to logfile\n")
            move_mode = int(move_mode)
            if move_mode == 0:
                move_arg = ["", move_mode, num]
            elif move_mode == 1:
                print("Since you want ", num, " drones, so please enter 3 positions")
                positions = input("Please enter in form of: (x1,y1) (x2,y2) ...\n")
                positions = positions.split(' ')
                move_arg = ["", move_mode, num] + positions

        ros_proc = sim_launch(num, './Drone')
        sys.path.insert(0, './Drone')
        import goto, move

        if ans == 1:
            rospy.init_node('Drone_GoTo', anonymous=True)
            pos1 = {'x': 5, 'y': -7, 'z': 10}
            pos2 = {'x': -5, 'y': -9, 'z': 10}
            pos3 = {'x': 5, 'y': 7, 'z': 10}
            goals = [pos1, pos2, pos3]
            navi = goto.GoTo(num, goals)
        elif ans == 2:
            move.main(move_arg)

        # Infinite loop like ros spin
        while(True):
            try:
                pass
            except rospy.ROSInterruptException:
                ros_proc.kill()
                os.system("killall -9 rosmaster")
                print("User pressed Ctrl-C, exit! ")
                exit(0)

    elif argv[1] == 'car':
        sys.path.insert(0, './F1tenth')
        import launch
        launch.launch(num, [[0, 0, 0.3], [0, 5, 0.3], [5, 5, 0.3], [5, 0, 0.3]])


def sim_launch(num, model):
    '''
    This function calls Gazebo simulator with the specified model
    :param num: number of models to be spawned
    :param model: the model that to be launched and spawned
    :return: ros_proc - the Gazebo-ROS process that is running
    '''
    sys.path.insert(0, model)
    import launch
    ros_proc = launch.launch(num, [[0, 0, 0.3], [0, 5, 0.3], [5, 5, 0.3], [5, 0, 0.3]])
    print("============= Simulator starts successful ================")
    time.sleep(max(num * 6, 10))
    return ros_proc


if __name__ == '__main__':
    main(sys.argv)
