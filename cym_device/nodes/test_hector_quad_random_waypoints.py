#!/usr/bin/env python3

import itertools
import math
import random
from typing import List, NamedTuple, Sequence, Tuple

from actionlib import SimpleActionClient
from geometry_msgs.msg import PoseStamped
from hector_uav_msgs.msg import PoseAction, PoseGoal
import rospy

Point = NamedTuple("Point", [('x', float), ('y', float), ('z', float)])


def dist(p0: Sequence[float], p1: Sequence[float]) -> float:
    return math.sqrt(sum((x0 - x1)**2 for x0, x1 in zip(p0, p1)))


def dist_to_corners(p: Point, rect: Tuple[Point, Point]) \
        -> Sequence[float]:
    return tuple(dist(p, (rect[i].x, rect[j].y, rect[k].z))
                 for i, j, k in itertools.product([0, 1], [0, 1], [0, 1]))


def in_rect(p: Point, rect: Tuple[Point, Point]):
    return all(x_min <= x <= x_max
               for x, x_min, x_max in zip(p, rect[0], rect[1]))


def gen_random_waypoints(n: int, center: Point, dist_range: Tuple[float, float],
                         area: Tuple[Point, Point]) \
        -> Sequence[Point]:
    """
    Generate a sequence of points following the rules below.

    + Odd index: a random point in the hollow sphere defined by the center and
      distance range and also within the area.
    + Even index: the center point

    Returns
    -------
    ret : Sequence [Point]
        A sequence of points
    """
    assert n > 0
    ret = []  # type: List[Point]
    r_min = dist_range[0]
    for _ in range(n // 2 + 1):
        r_max = min(dist_range[1], max(dist_to_corners(center, area)))
        assert r_min <= r_max
        r = random.uniform(r_min, r_max)

        # Sphere Point picking.
        # See https://mathworld.wolfram.com/SpherePointPicking.html
        x1, x2 = 1, 1
        while True:
            x1 = random.uniform(-1.0, 1.0)
            x2 = random.uniform(-1.0, 1.0)
            if x1**2 + x2**2 >= 1:
                continue  # Pick again
            unit_x = 2 * x1 * math.sqrt(1 - x1**2 - x2**2)
            unit_y = 2 * x2 * math.sqrt(1 - x1**2 - x2**2)
            unit_z = 1 - 2 * (x1**2 + x2**2)
            pick = Point(center.x + r*unit_x,
                         center.y + r*unit_y,
                         center.z + r*unit_z)
            if not in_rect(pick, area):
                continue  # Pick again
            ret.append(pick)
            ret.append(center)
            break
    assert len(ret) == (n // 2 + 1) * 2
    ret = ret[:n]  # Take only first n elements

    assert all(in_rect(p, area) for p in ret)

    return ret


def main():
    rospy.init_node("random_waypoints", anonymous=True)
    num_waypoints = rospy.get_param("~num_waypoints", 10)
    time_itvl_min = rospy.get_param("~time_itvl_min", 1.0)
    center = Point(*rospy.get_param("~center", [0.0, 0.0, 2.5]))
    dist_range = (rospy.get_param("~dist_min", 1.0), rospy.get_param("~dist_max", 5.0))
    area = (Point(rospy.get_param("~x_min", 0.0), rospy.get_param("~y_min", 0.0), rospy.get_param("~z_min", 0.5)),
            Point(rospy.get_param("~x_max", 4.0), rospy.get_param("~y_max", 4.0), rospy.get_param("~z_max", 4.5)))
    waypoint_seq = gen_random_waypoints(num_waypoints, center, dist_range, area)

    pose_topic = rospy.resolve_name("action/pose")
    pose_client = SimpleActionClient(pose_topic, PoseAction)

    reached = False

    def action_pose_done_cb(goal_state, goal_result):
        nonlocal reached
        rospy.loginfo("Reached")
        reached = True

    waypoint_iter = iter(waypoint_seq)
    curr_wp = next(waypoint_iter, None)
    rate = rospy.Rate(100)
    is_driving = False
    next_waypoint_time = 0.0
    while not rospy.is_shutdown() and curr_wp is not None:
        rate.sleep()
        if not is_driving:
            if rospy.get_time() < next_waypoint_time:
                continue
            # else
            pose_client.wait_for_server()
            tgt = PoseStamped()
            tgt.header.frame_id = "world"
            tgt.pose.position.x = curr_wp.x
            tgt.pose.position.y = curr_wp.y
            tgt.pose.position.z = curr_wp.z
            rospy.loginfo("Sending pose goal\n %s" % str(tgt))
            pose_client.send_goal(PoseGoal(target_pose=tgt),
                                  done_cb=action_pose_done_cb)
            is_driving = True
            next_waypoint_time = rospy.get_time() + time_itvl_min
        else:  # Wait until reaching the waypoint
            if not reached:
                pass
            else:
                is_driving = False
                reached = False
                curr_wp = next(waypoint_iter, None)


if __name__ == "__main__":

    try:
        main()
    except rospy.exceptions.ROSInterruptException:
        rospy.loginfo("Shutting down TestHectorQuadRandomWaypoints")
