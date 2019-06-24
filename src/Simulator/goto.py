import argparse
import _thread

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-c", "--car", help="Number of cars to be driven", type=int)
    parser.add_argument("-d", "--drone", help="Number of drones to be driven", type=int)
    parser.add_argument("-G", "--goal", nargs="+", help="Goal locations of the models")


if __name__ == '__main__':
    main()