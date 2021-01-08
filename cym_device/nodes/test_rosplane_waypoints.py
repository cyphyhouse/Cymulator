#!/usr/bin/env python3

import math
from typing import NamedTuple, Sequence

import rospy
from rosplane_msgs.msg import Waypoint


Point = NamedTuple("Point", [('x', float), ('y', float), ('z', float)])


def dist(p0: Sequence[float], p1: Sequence[float]) -> float:
    return math.sqrt(sum((x0 - x1)**2 for x0, x1 in zip(p0, p1)))


def to_rosplane_waypoint(p: Point, chi_d: float = math.radians(45),
                         chi_valid: bool = True,
                         Va_d: float = 12, set_current: bool = True,
                         clear_wp_list: bool = False) -> Waypoint:
    return Waypoint(
        w=tuple(p), chi_d=chi_d, chi_valid=chi_valid, Va_d=Va_d,
        set_current=set_current, clear_wp_list=clear_wp_list
    )


def gen_waypoints(num_waypoints, xy_mode, z_mode) -> Sequence:
    return [(Point(200, 0, -50), math.radians(45)),
            (Point(0, 200, -50), math.radians(45)),
            (Point(200, 200, -50), math.radians(225))]


def main():
    rospy.init_node("rosplane_waypoints", anonymous=True)
    num_waypoints = rospy.get_param("~num_waypoints", 10)
    time_itvl_min = rospy.get_param("~time_itvl_min", 10.0)
    center = Point(*rospy.get_param("~center", [0.0, 0.0, 2.5]))
    dist_range = (rospy.get_param("~dist_min", 1.0), rospy.get_param("~dist_max", 5.0))
    area = (Point(rospy.get_param("~x_min", 0.0), rospy.get_param("~y_min", 0.0), rospy.get_param("~z_min", 0.5)),
            Point(rospy.get_param("~x_max", 4.0), rospy.get_param("~y_max", 4.0), rospy.get_param("~z_max", 4.5)))
    xy_mode = rospy.get_param("~xy_mode", "RSR")
    z_mode = rospy.get_param("~z_mode", "low")

    waypoint_topic = rospy.resolve_name("waypoint_path")
    waypoint_pub = rospy.Publisher(waypoint_topic, Waypoint, queue_size=10)

    waypoint_seq = gen_waypoints(num_waypoints, xy_mode, z_mode)

    rospy.sleep(2.0)
    waypoint_pub.publish(to_rosplane_waypoint(p=waypoint_seq[0][0],
                                              chi_d=waypoint_seq[0][1],
                                              set_current=True))
    for point, chi_d in waypoint_seq[1:]:
        rospy.sleep(2.0)
        waypoint_pub.publish(to_rosplane_waypoint(p=point, chi_d=chi_d,
                                                  set_current=False))

    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.exceptions.ROSInterruptException:
        rospy.loginfo("Shutting down TestROSplaneWaypoints")
