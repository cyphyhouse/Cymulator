import rospy
import argparse
from nav_msgs.msg import Odometry
import itertools
from math import sqrt
import goto


def distanceBetween(a, b):
    return sqrt((a[0]-b[0])*(a[0]-b[0]) + (a[1]-b[1])*(a[1]-b[1]))


def line_formation(num):
    # Subscribe to know locations of all models
    rospy.init_node("Line_Formation", anonymous=True)
    droneLocs = {}

    # Get locations of all drones
    for i in range(num):
        my_number = 'drone' + str(i+1)
        msg = rospy.wait_for_message(my_number + "/ground_truth/state", Odometry)
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        droneLocs[i+1] = (x, y, z)

    print(droneLocs)

    comb = itertools.combinations(range(1, num+1), 2)
    maxDist = 0
    start, end = 0, 0
    for c in comb:
        a, b = c
        dist = distanceBetween(droneLocs[a], droneLocs[b])
        if dist > maxDist:
            maxDist = dist
            start = a
            end = b

    droneList = list(range(1, num+1))
    droneList.remove(start)
    droneList.remove(end)

    diff_x = droneLocs[start][0] - droneLocs[end][0]
    diff_y = droneLocs[start][1] - droneLocs[end][1]
    if diff_x < 0:
        diff_x = -diff_x
    if diff_y < 0:
        diff_y = -diff_y

    goals = []
    # TODO: Add goals for drones that need to move


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-d', '--drone', help="Number of drones to do line formation", type=int)
    args = parser.parse_args()
    num_drones = args.drone
    line_formation(num_drones)
