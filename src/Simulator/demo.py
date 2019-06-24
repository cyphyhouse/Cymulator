import sys
import rospy
import os
from util import parse_init_pose, parse_goal_pose,sim_launch
import time


def main(argv):
    if len(argv) < 4:
        print("Please enter arguments in form of: python3 demo.py MODEL_TYPE NUMBER_OF_MODEL DEMO_MODE")
        exit(0)

    _, model, num, mode = argv
    num = int(num)
    mode = int(mode)

    init_poses = [(0,0,0.3), (0,2,0.3), (0,4,0.3), (0,6,0.3)]
    drone_goals = [(10,10,10), (10,-10,10), (-10,10,10), (-10,-10,10)]
    car_goals = [(10, 10), (10, -10), (-10, 10), (-10, -10)]

    if model == 'drone':
        print("Generate drones")
        proc = sim_launch(num, './Drone', init_poses)
        time.sleep(max(num * 6, 10))
        sys.path.insert(0, './Drone')
        import goto, move
        if mode == 1:
            rospy.init_node('Drone_GoTo', anonymous=True)
            goto.GoTo(num, drone_goals)

    elif model == 'car' or model == 'f1tenth':
        print("Generate cars")
        proc = sim_launch(num, './F1tenth', init_poses)
        time.sleep(max(num * 6, 10))
        sys.path.insert(0, './F1tenth')
        import goto, move
        if mode == 1:
            rospy.init_node('F1tenth_GoTo', anonymous=True)
            goto.GoTo(num, car_goals)

    else:
        print("Invalid model type")
        exit(0)

    # Infinite loop like ros spin
    while True:
        try:
            # print("PASS")
            pass
        except rospy.ROSInterruptException:
            os.system("killall -9 rosmaster")
            os.system("killall -9 gzserver")
            os.system("killall -9 python")
            proc.kill()
            print("User pressed Ctrl-C, exit! ")
            sys.exit()


if __name__ == '__main__':
    # python3 demo.py MODEL_TYPE NUMBER_OF_MODEL DEMO_MODE
    main(sys.argv)
