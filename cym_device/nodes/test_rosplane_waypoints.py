#!/usr/bin/env python3

import math
from typing import NamedTuple, Sequence

import rospy
from rosplane_msgs.msg import Waypoint


Point = NamedTuple("Point", [('x', float), ('y', float), ('z', float)])


def dist(p0: Sequence[float], p1: Sequence[float]) -> float:
    return math.sqrt(sum((x0 - x1)**2 for x0, x1 in zip(p0, p1)))


class ToROSplaneWaypoint:
    def __init__(self, init_pose=(0.0, 0.0, 0.3, 0.0)):
        self._init_pose = tuple(init_pose)

    def to_rosplane_waypoint(self, p: Point, chi_d: float = math.radians(45),
                             chi_valid: bool = False,
                             Va_d: float = 12, set_current: bool = True,
                             clear_wp_list: bool = False) -> Waypoint:
        n =   p[0] - self._init_pose[0]
        e = -(p[1] - self._init_pose[1])
        d = -(p[2] - self._init_pose[2])
        return Waypoint(
            w=(n, e, d), chi_d=chi_d, chi_valid=chi_valid, Va_d=Va_d,
            set_current=set_current, clear_wp_list=clear_wp_list
        )


def gen_waypoints(waypoints: Sequence[Sequence[float]]) -> Sequence:
    for p in waypoints:
        if len(p) != 4:
            rospy.logwarn("The size of waypoint %s != 4. "
                          "Discard all waypoints." % str(p))
            return []

    return [(Point(*p[0:3]), math.radians(p[3])) for p in waypoints]


def main():
    rospy.init_node("rosplane_waypoints", anonymous=True)
    init_pose = rospy.get_param("~init_pose", [0.0, 0.0, 0.3, 0.0])
    waypoints = rospy.get_param("~waypoints", [])

    waypoint_topic = rospy.resolve_name("waypoint_path")
    waypoint_pub = rospy.Publisher(waypoint_topic, Waypoint, queue_size=10)

    waypoint_seq = gen_waypoints(waypoints)

    rospy.sleep(2.0)
    converter = ToROSplaneWaypoint(init_pose)

    waypoint_pub.publish(converter.to_rosplane_waypoint(p=waypoint_seq[0][0],
                                                        chi_d=waypoint_seq[0][1],
                                                        set_current=True))
    for point, chi_d in waypoint_seq[1:]:
        rospy.sleep(2.0)
        waypoint_pub.publish(converter.to_rosplane_waypoint(p=point, chi_d=chi_d,
                                                            set_current=False))

    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.exceptions.ROSInterruptException:
        rospy.loginfo("Shutting down TestROSplaneWaypoints")
