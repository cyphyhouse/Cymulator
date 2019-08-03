#!/usr/bin/env python3

"""ROS node to drive a drone following given waypoints"""

from copy import deepcopy
from queue import Queue
from typing import Tuple

from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from hector_uav_msgs.srv import EnableMotors
import rospy
from std_msgs.msg import String


class __DroneStates:
    def __init__(self, init_pose, init_twist):
        assert init_pose
        assert init_twist
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

    def pub_values(self) -> Tuple[str, Twist]:
        if not self._waypoints:  # No waypoint to go
            return "True", Twist()
        # else
        # TODO figure out how not to store current waypoint
        curr_waypoint = self._waypoints.get()
        goal = curr_waypoint.pose.position
        curr = self._pose.pose.position

        # FIXME pass a function to decide whether the waypoint is reached?
        dist_2 = (goal.x - curr.x)**2 + \
                 (goal.y - curr.y)**2 + \
                 (goal.z - curr.z)**2
        return dist_2 < (0.25**2)  # sqrt(dist_2) < 0.25

    def _cmd_vel(self) -> Twist:
        if self._is_reached():  # Already reached
            return Twist()  # Default is 0 for every field
        goal = self._waypoints[0].pose.position
        curr = self._pose.pose.position

        # FIXME pass a function to calculate velocity?
        def cal_vel(diff: float) -> float:
            if abs(diff) > 0.5:
                ret = 0.5 if diff > 0.0 else -0.5
            else:
                ret = 0.05 if diff > 0.0 else -0.05
            return ret

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
    init_pose = rospy.wait_for_message(pose_topic_name, PoseStamped)
    init_twist = rospy.wait_for_message(twist_topic_name, TwistStamped)

    ds = __DroneStates(init_pose, init_twist)
    # For positioning
    sub_pose = rospy.Subscriber(pose_topic_name, PoseStamped, ds.store_pose)
    sub_twist = rospy.Subscriber(twist_topic_name, TwistStamped, ds.store_twist)
    # For middleware
    sub_waypoint = rospy.Subscriber("~waypoint", PoseStamped, ds.store_waypoint)
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

    rate = rospy.Rate(10)  # 10 hz TODO Pass sleep rate as a parameter?
    while not rospy.is_shutdown():
        # Simple controller code for drones # TODO Need better controller

        is_reached_str, cmd_vel = ds.pub_values()
        pub_reached.publish(is_reached_str)
        pub_cmd_vel.publish(cmd_vel)

        try:
            rate.sleep()
        except rospy.exceptions.ROSInterruptException:
            rospy.loginfo("Shutting down CymVRPN")
            break


if __name__ == "__main__":
    import sys
    main(sys.argv)
