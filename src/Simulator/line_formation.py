import rospy
import argparse
from nav_msgs.msg import Odometry
import itertools
from math import sqrt
import goto


def distanceBetween(a, b):
    return sqrt((a[0]-b[0])*(a[0]-b[0]) + (a[1]-b[1])*(a[1]-b[1]))


def line(k, x1, y1, x):
    return k*(x-x1) + y1


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
    # Find 2 drones with largest distance
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

    # Get locations where each drone should fly to
    x1 = droneLocs[start][0]
    y1 = droneLocs[start][1]
    x2 = droneLocs[end][0]
    y2 = droneLocs[end][1]
    k = (y2-y1)/(x2-x1)
    goals = []
    x = x1 if x1 < x2 else x2
    diff_x = abs(x2-x1) if x2 > x1 else abs(x1-x2)
    diff_x = diff_x/num
    for i in range(num):
        if i+1 == start:
            goals.append([x1, y1, 5])
            continue
        x += diff_x
        y = line(k, x1, y1, x)
        goals.append([x, y, 5])

    # Call goto method to drive drones
    goto.main(num, 0, goals)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-d', '--drone', help="Number of drones to do line formation", type=int)
    args = parser.parse_args()
    num_drones = args.drone
    line_formation(num_drones)
