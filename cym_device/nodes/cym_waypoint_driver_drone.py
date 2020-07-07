#!/usr/bin/env python3

"""ROS node to drive a drone following given waypoints"""

from copy import deepcopy
from enum import Enum
from queue import Queue
from threading import RLock

from actionlib import SimpleActionClient
from geometry_msgs.msg import PoseStamped
from hector_uav_msgs.msg import PoseAction, PoseGoal, \
    LandingAction, LandingGoal, TakeoffAction, TakeoffGoal

import rospy
from std_msgs.msg import String


class ReachedEnum(Enum):
    NO = 0
    YES = 1
    YES_AND_REPORT = 2


class __DroneStates:

    def __init__(self):

        # Thread safe Queue is required because pub and sub can be in
        # different threads
        self._waypoints = Queue()  # type: Queue

        self._var_lock = RLock()
        self._curr_waypoint = None
        self._reached = ReachedEnum.YES

    def store_waypoint(self, msg: PoseStamped) -> None:
        """
        Callback function to get drone's next waypoint
        :param msg: PoseStamped type msg that contains the new waypoint
        :return: Nothing
        """
        rospy.logdebug("Received waypoint %s" % str(msg.pose.position))
        self._waypoints.put(msg)

    def set_curr_waypoint(self):
        with self._var_lock:
            assert not self._waypoints.empty()
            self._curr_waypoint = self._waypoints.get()
            self._reached = ReachedEnum.NO

    @property
    def curr_waypoint(self):
        with self._var_lock:
            return self._curr_waypoint

    def reset_curr_waypoint(self):
        with self._var_lock:
            assert self._curr_waypoint is not None
            assert isinstance(self._curr_waypoint, PoseStamped)
            if self._curr_waypoint.header.frame_id == "1":
                self._reached = ReachedEnum.YES_AND_REPORT
            else:
                self._reached = ReachedEnum.YES
            self._curr_waypoint = None

    @property
    def reached(self) -> ReachedEnum:
        with self._var_lock:
            return self._reached

    def report_reached(self) -> str:
        with self._var_lock:
            if self._reached == ReachedEnum.YES_AND_REPORT:
                self._reached = ReachedEnum.YES
                return str(True)
        return ""

    def target_pose(self) -> PoseStamped:
        with self._var_lock:
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
    rospy.init_node('waypoint_node')
    # Register publishers first
    pub_reached = rospy.Publisher("~reached", String,
                                  queue_size=1)  # FIXME decide queue_size

    # Register subscribers
    ds = __DroneStates()
    # For middleware
    waypoint_topic_name = "~waypoint"
    _ = rospy.Subscriber(waypoint_topic_name, PoseStamped, ds.store_waypoint)

    # Register actionlib clients
    takeoff_topic = rospy.resolve_name("action/takeoff")
    takeoff_client = SimpleActionClient(takeoff_topic, TakeoffAction)
    landing_topic = rospy.resolve_name("action/landing")
    landing_client = SimpleActionClient(landing_topic, LandingAction)

    pose_topic = rospy.resolve_name("action/pose")
    pose_client = SimpleActionClient(pose_topic, PoseAction)

    def action_pose_done_cb(goal_state, goal_result):
        rospy.logdebug("Reached\n %s" % str(ds.curr_waypoint.pose.position))
        ds.reset_curr_waypoint()

    def shutdown() -> None:  # TODO Better place for this code
        """Stop the drone when this ROS node shuts down"""
        # TODO Safe landing
        pass

    rospy.on_shutdown(shutdown)

    # TODO Wait for hector quadrotor controllers to spawn
    rospy.sleep(1)

    rate = rospy.Rate(100)  # 100Hz TODO Pass sleep rate as a parameter?

    is_driving = False
    while not rospy.is_shutdown():
        rate.sleep()
        # Simple controller code for drones # TODO Need better controller
        if not is_driving:  # IDLE
            if ds._waypoints.empty():  # FIXME accessing protected member
                pass  # Keep idling
            else:
                ds.set_curr_waypoint()
                pose_client.wait_for_server()

                pose_goal = PoseGoal(target_pose=ds.target_pose())
                rospy.logdebug("Sending pose goal\n %s" % str(pose_goal))

                pose_client.send_goal(PoseGoal(target_pose=ds.target_pose()),
                                      done_cb=action_pose_done_cb)
                is_driving = True
        else:  # DRIVING
            if ds.reached == ReachedEnum.NO:
                pass  # Keep driving
            else:
                if ds.reached == ReachedEnum.YES_AND_REPORT:
                    pub_reached.publish(ds.report_reached())
                is_driving = False



if __name__ == "__main__":
    import sys

    try:
        main(rospy.myargv(argv=sys.argv))
    except rospy.exceptions.ROSInterruptException:
        rospy.loginfo("Shutting down CymWaypointDriveDrone")
