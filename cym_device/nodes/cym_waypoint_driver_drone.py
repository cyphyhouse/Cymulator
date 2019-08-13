#!/usr/bin/env python3

"""ROS node to drive a drone following given waypoints"""

from copy import deepcopy
from math import copysign
from queue import Queue
from typing import Tuple

from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from hector_uav_msgs.srv import EnableMotors
import rospy
from std_msgs.msg import String


class __DroneStates:
    def __init__(self,
                 init_pose: PoseStamped,
                 init_twist: TwistStamped,
                 init_waypoint: PoseStamped):
        assert init_pose
        assert init_twist
        assert init_waypoint
        self._curr_waypoint = init_waypoint
        self._waypoints = Queue()  # Thread safe Queue is required because pub and sub can be in different threads
        self._pose = init_pose
        self._twist = init_twist

    def store_waypoint(self, msg: PoseStamped) -> None:
        """
        Callback function to get drone's next waypoint
        :param msg: PoseStamped type msg that contains the new waypoint
        :return: Nothing
        """
        self._waypoints.put(msg)

    def store_pose(self, msg: PoseStamped) -> None:
        """
        Callback function to get drone's current pose
        :param msg: PoseStamped type msg
        :return: Nothing
        """
        self._pose = deepcopy(msg)  # In case `msg` is overwritten

    def store_twist(self, msg: PoseStamped) -> None:
        """
        Callback function to get drone's current twist
        :param msg: TwistStamped type msg
        :return: Nothing
        """
        self._twist = deepcopy(msg)  # In case `msg` is overwritten

    def is_reached(self) -> bool:
        goal = self._curr_waypoint.pose.position
        curr = self._pose.pose.position

        # FIXME pass a function to decide whether the waypoint is reached?
        dist_2 = (goal.x - curr.x) ** 2 + \
                 (goal.y - curr.y) ** 2 + \
                 (goal.z - curr.z) ** 2
        return dist_2 < (0.25 ** 2)  # sqrt(dist_2) < 0.25

    def set_next_waypoint(self):
        if not self._waypoints.empty():
            self._curr_waypoint = self._waypoints.get()
        # else: _curr_waypoint remains so is_reached should be true

    def cmd_vel(self) -> Twist:
        if self.is_reached():
            return Twist()  # Stop movement
        # else:
        goal = self._curr_waypoint.pose.position
        curr = self._pose.pose.position

        # FIXME pass a function to calculate velocity?
        def cal_vel(diff: float) -> float:
            magnitude = max(abs(diff), 0.5)
            return copysign(magnitude, diff)

        move_cmd = Twist()
        move_cmd.linear.x = cal_vel(goal.x - curr.x)
        move_cmd.linear.y = cal_vel(goal.y - curr.y)
        move_cmd.linear.z = cal_vel(goal.z - curr.z)
        return move_cmd


def main(argv) -> None:
    """
     Main entry point
        This design of putting all pub/sub in main function is intentional
        since we want to avoid pub/sub being instantiated more than once.
    :param argv:
    """
    tracker_id = argv[1]  # TODO Read from parameter server instead

    rospy.init_node('waypoint_node')
    # Wait for positioning system to start
    pose_topic_name = "/vrpn_client_node/" + tracker_id + "/pose"
    twist_topic_name = "/vrpn_client_node/" + tracker_id + "/twist"
    waypoint_topc_name = "~waypoint"
    init_pose = rospy.wait_for_message(pose_topic_name, PoseStamped)
    init_twist = rospy.wait_for_message(twist_topic_name, TwistStamped)
    init_waypoint = rospy.wait_for_message(waypoint_topc_name, PoseStamped)

    ds = __DroneStates(init_pose, init_twist, init_waypoint)
    # For positioning
    _ = rospy.Subscriber(pose_topic_name, PoseStamped, ds.store_pose)
    _ = rospy.Subscriber(twist_topic_name, TwistStamped, ds.store_twist)
    # For middleware
    _ = rospy.Subscriber(waypoint_topc_name, PoseStamped, ds.store_waypoint)
    pub_reached = rospy.Publisher("~reached", String, queue_size=1)  # FIXME how to decide queue_size
    # For driving the simulated drone
    pub_cmd_vel = rospy.Publisher("cmd_vel", Twist, queue_size=10)  # FIXME how to decide queue_size

    # Enable motors using ROS service
    service_name = "enable_motors"
    rospy.wait_for_service(service_name)
    try:
        # Set 'enable_motors' to be True
        enable_motor = rospy.ServiceProxy(service_name, EnableMotors)
        resp = enable_motor(True)
        if not resp.success:
            pass  # TODO What to do if motor is not enabled
    except rospy.ServiceException as e:
        rospy.loginfo("Service call failed with %s", e)

    def shutdown() -> None:  # TODO Better place for this code
        """Stop the drone when this ROS node shuts down"""
        pub_cmd_vel.publish(Twist())  # Default Twist will stop the drone
        # TODO Safe landing
        rospy.loginfo("Stop the drone")

    rospy.on_shutdown(shutdown)

    rate = rospy.Rate(10)  # 10 hz TODO Pass sleep rate as a parameter?
    while not rospy.is_shutdown():
        # Simple controller code for drones # TODO Need better controller

        pub_reached.publish(str(ds.is_reached()))
        pub_cmd_vel.publish(ds.cmd_vel())
        if ds.is_reached():
            ds.set_next_waypoint()

        rate.sleep()


if __name__ == "__main__":
    import sys

    try:
        main(rospy.myargv(argv=sys.argv))
    except rospy.exceptions.ROSInterruptException:
        rospy.loginfo("Shutting down CymWaypointDriveDrone")
