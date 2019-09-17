#!/usr/bin/env python3

"""ROS node to drive a drone following given waypoints"""

from copy import deepcopy
from math import copysign
from queue import Queue
from threading import RLock
from typing import Tuple

from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from hector_uav_msgs.srv import EnableMotors
import rospy
from std_msgs.msg import Bool, String


class __DroneStates:

    def __init__(self,
                 init_pose: PoseStamped,
                 init_twist: TwistStamped):
        assert init_pose
        assert init_twist

        self._pose_lock = RLock()
        self._pose = init_pose
        self._twist_lock = RLock()
        self._twist = init_twist

        self._curr_waypoint = init_pose
        self._waypoints = Queue()  # Thread safe Queue is required because pub and sub can be in different threads

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
        with self._pose_lock:
            self._pose = deepcopy(msg)  # In case `msg` is overwritten

    def store_twist(self, msg: PoseStamped) -> None:
        """
        Callback function to get drone's current twist
        :param msg: TwistStamped type msg
        :return: Nothing
        """
        with self._twist_lock:
            self._twist = deepcopy(msg)  # In case `msg` is overwritten

    def has_reached(self) -> bool:
        assert self._curr_waypoint
        goal = self._curr_waypoint.pose.position

        with self._pose_lock:
            curr = self._pose.pose.position

        # FIXME pass a function to decide whether the waypoint is reached?
        dist_2 = (goal.x - curr.x) ** 2 + \
                 (goal.y - curr.y) ** 2 + \
                 (goal.z - curr.z) ** 2
        return dist_2 < (0.25 ** 2)  # sqrt(dist_2) < 0.25

    def set_curr_waypoint(self):
        assert not self._waypoints.empty()
        self._curr_waypoint = self._waypoints.get()

    @property
    def curr_waypoint(self):
        return self._curr_waypoint

    def reset_curr_waypoint(self):
        self._curr_waypoint = None

    def cmd_pose(self) -> PoseStamped:
        assert self._curr_waypoint
        pose_cmd = deepcopy(self._curr_waypoint)
        pose_cmd.header.frame_id = "world"
        return pose_cmd


def main(argv) -> None:
    """
     Main entry point
        This design of putting all pub/sub in main function is intentional
        since we want to avoid pub/sub being instantiated more than once.
    :param argv:
    """
    tracker_id = argv[1]  # TODO Read from parameter server instead

    rospy.init_node('waypoint_node')
    # Register publishers first
    pub_reached = rospy.Publisher("~reached", String, queue_size=1)  # FIXME how to decide queue_size
    # For driving the simulated drone
    # FIXME Should use actionlib server provided by hector quadrotor instead
    pub_cmd_pose = rospy.Publisher("command/pose", PoseStamped, queue_size=10)  # FIXME how to decide queue_size
    pub_estop = rospy.Publisher("estop", Bool, queue_size=1)

    # Register subscribers
    # Wait for positioning system to start
    pose_topic_name = "/vrpn_client_node/" + tracker_id + "/pose"
    twist_topic_name = "/vrpn_client_node/" + tracker_id + "/twist"
    waypoint_topc_name = "~waypoint"
    init_pose = rospy.wait_for_message(pose_topic_name, PoseStamped)
    init_twist = rospy.wait_for_message(twist_topic_name, TwistStamped)

    ds = __DroneStates(init_pose, init_twist)
    # For positioning
    _ = rospy.Subscriber(pose_topic_name, PoseStamped, ds.store_pose)
    _ = rospy.Subscriber(twist_topic_name, TwistStamped, ds.store_twist)
    # For middleware
    _ = rospy.Subscriber(waypoint_topc_name, PoseStamped, ds.store_waypoint)

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
        pub_estop.publish(Bool(True))  # Emergency stop
        # TODO Safe landing
        rospy.loginfo("Stop the drone")

    rospy.on_shutdown(shutdown)

    # TODO Wait for hector quadrotor controllers to spawn
    rospy.sleep(1)

    rate = rospy.Rate(50)  # TODO Pass sleep rate as a parameter?
    is_driving = False
    while not rospy.is_shutdown():
        rate.sleep()
        # Simple controller code for drones # TODO Need better controller
        if not is_driving:  # IDLE
            if not ds._waypoints.empty():  # FIXME accessing protected member
                ds.set_curr_waypoint()
                is_driving = True
            # else: is_driving = False  # Keep idling
        else:  # DRIVING
            pub_cmd_pose.publish(ds.cmd_pose())
            if ds.has_reached():
                assert ds.curr_waypoint
                if ds.curr_waypoint.header.frame_id == "1":
                    pub_reached.publish(str(True))
                is_driving = False
            # else: is_driving = True  # Keep driving


if __name__ == "__main__":
    import sys

    try:
        main(rospy.myargv(argv=sys.argv))
    except rospy.exceptions.ROSInterruptException:
        rospy.loginfo("Shutting down CymWaypointDriveDrone")
