import sys


def main(argv):
    if len(argv) < 3:
        print("Must choose a model and specify the number")
        exit(0)
    if argv[1] == 'drone':
        from Drone import launch, goto, drone
        launch.launch(int(argv[2]), [[0, 0, 0.3], [0, 5, 0.3], [5, 5, 0.3], [5, 0, 0.3]])

    elif argv[1] == 'car':
        from F1tenth import launch
        launch.launch(int(argv[2]), [[0, 0, 0.3], [0, 5, 0.3], [5, 5, 0.3], [5, 0, 0.3]])


if __name__ == '__main__':
    main(sys.argv)