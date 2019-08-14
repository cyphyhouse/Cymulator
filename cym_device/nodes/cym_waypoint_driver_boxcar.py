#!/usr/bin/env python3

"""ROS node to drive a boxcar following given waypoints"""

from copy import deepcopy
import math

from geometry_msgs.msg import PoseStamped, Twist, TwistStamped, Quaternion
import rospy
from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion


class __DeviceState(object):
    HALF_CYCLE = 5.0  # sec
    ABS_VEL = 2.0
    ABS_ANG_VEL = 2.0 * math.pi / 10

    def __init__(self,
                 init_pose: PoseStamped,
                 init_twist: TwistStamped):
        assert init_pose
        assert init_twist
        self._pose = init_pose
        self._twist = init_twist
        self._cycle_begin = rospy.get_time()

    def store_pose(self, msg: PoseStamped) -> None:
        """
        Callback function to get device's current pose
        :param msg: PoseStamped type msg
        :return: Nothing
        """
        self._pose = deepcopy(msg)  # In case `msg` is overwritten

    def store_twist(self, msg: PoseStamped) -> None:
        """
        Callback function to get device's current twist
        :param msg: TwistStamped type msg
        :return: Nothing
        """
        self._twist = deepcopy(msg)  # In case `msg` is overwritten

    def is_first_half(self) -> bool:
        return rospy.get_time() < self._cycle_begin + self.HALF_CYCLE

    def cmd_vel(self) -> Twist:
        if rospy.get_time() >= self._cycle_begin + 2 * self.HALF_CYCLE:
            # Roughly finished a cycle. Reset
            self._cycle_begin = rospy.get_time()

        vel = self.ABS_VEL if self.is_first_half() else -self.ABS_VEL
        ang_vel = self.ABS_ANG_VEL if self.is_first_half() else -self.ABS_ANG_VEL

        theta = euler_from_quaternion(
            [self._pose.pose.orientation.w,
             self._pose.pose.orientation.x,
             self._pose.pose.orientation.y,
             self._pose.pose.orientation.z,
             ])

        cmd = Twist()
        cmd.linear.x = vel * math.sin(theta[2])
        cmd.linear.y = vel * math.cos(theta[2])
        cmd.angular.z = ang_vel
        return cmd


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
    # FIXME Read topic names from parameter server
    pose_topic_name = "/vrpn_client_node/" + tracker_id + "/pose"
    twist_topic_name = "/vrpn_client_node/" + tracker_id + "/twist"

    init_pose = rospy.wait_for_message(pose_topic_name, PoseStamped)
    init_twist = rospy.wait_for_message(twist_topic_name, TwistStamped)

    ds = __DeviceState(init_pose, init_twist)
    # For positioning
    _ = rospy.Subscriber(pose_topic_name, PoseStamped, ds.store_pose)
    _ = rospy.Subscriber(twist_topic_name, TwistStamped, ds.store_twist)
    # For driving the simulated drone
    pub_cmd_vel = rospy.Publisher("cmd_vel", Twist, queue_size=10)  # FIXME how to decide queue_size

    def shutdown() -> None:  # TODO Better place for this code
        """Stop the drone when this ROS node shuts down"""
        pub_cmd_vel.publish(Twist())  # Default Twist will stop the drone
        # TODO Safe landing
        rospy.loginfo("Stop the drone")

    rospy.on_shutdown(shutdown)

    rate = rospy.Rate(100)  # 100 Hz
    while not rospy.is_shutdown():
        pub_cmd_vel.publish(ds.cmd_vel())
        rate.sleep()


if __name__ == "__main__":
    import sys

    try:
        main(rospy.myargv(argv=sys.argv))
    except rospy.exceptions.ROSInterruptException:
        rospy.loginfo("Shutting down CymWaypointDriveBoxcar")
