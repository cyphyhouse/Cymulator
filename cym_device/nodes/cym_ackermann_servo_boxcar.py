#!/usr/bin/env python3

from copy import deepcopy
import math
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped, Quaternion
from std_msgs.msg import Float64, Int32, String


import rospy

from scipy.spatial.transform import Rotation as R



class __DeviceState(object):
    CAR_LENGTH = 0.5  # FIXME Set car length from parameter server?

    def __init__(self, tracker_id: String):
        self.twist = Twist()
        self.tracker_id = tracker_id

    def set_state(self, msg: AckermannDriveStamped) -> ModelState:
        rospy.wait_for_service('/gazebo/get_model_state')
        modelInfo = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        currState = modelInfo(model_name=self.tracker_id)
        # TODO Handle not success
        vel = msg.drive.speed
        steering_angle = msg.drive.steering_angle

        quaternion = (currState.pose.orientation.x,
                      currState.pose.orientation.y,
                      currState.pose.orientation.z,
                      currState.pose.orientation.w)

        r = R.from_quat(quaternion)
        euler = r.as_euler('zyx', degrees=True)

        self.twist.linear.x = vel * math.cos(math.radians(euler[0]))
        self.twist.linear.y = vel * math.sin(math.radians(euler[0]))
        self.twist.angular.z = vel * (math.tan(steering_angle)/self.CAR_LENGTH)


def main(argv) -> None:
    """
     Main entry point
        This design of putting all pub/sub in main function is intentional
        since we want to avoid pub/sub being instantiated more than once.
    :param argv:
    """
    tracker_id = argv[1]  # TODO Read from parameter server instead

    rospy.init_node('ackermann_servo_node')
    # FIXME Read topic names from parameter server
    ackermann_topic_name = "ackermann_cmd_mux/output"

    ds = __DeviceState(tracker_id)
    # For ackermann
    _ = rospy.Subscriber(ackermann_topic_name, AckermannDriveStamped, ds.set_state)

    rospy.wait_for_service('/gazebo/get_model_state')
    modelInfo = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

    # For driving the simulated drone
    pub_model_state = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=10)  # FIXME how to decide queue_size

    def shutdown() -> None:  # TODO Better place for this code
        """Stop the drone when this ROS node shuts down"""
        state = ModelState()
        state.model_name = tracker_id
        pub_model_state.publish(state)  # Default Twist will reset the car
        # TODO Safe Termination
        rospy.loginfo("Stop the car")

    rospy.on_shutdown(shutdown)

    rate = rospy.Rate(100)  # 100 Hz
    while not rospy.is_shutdown():
        rospy.wait_for_service('/gazebo/get_model_state')
        state = modelInfo(model_name=tracker_id)

        newState = ModelState()
        newState.model_name = tracker_id
        newState.pose.position.x = state.pose.position.x
        newState.pose.position.y = state.pose.position.y
        newState.pose.orientation.x = state.pose.orientation.x
        newState.pose.orientation.y = state.pose.orientation.y
        newState.pose.orientation.z = state.pose.orientation.z
        newState.pose.orientation.w = state.pose.orientation.w
        newState.twist.linear.x = ds.twist.linear.x
        newState.twist.linear.y = ds.twist.linear.y
        newState.twist.angular.z = ds.twist.angular.z

        pub_model_state.publish(newState)
        rate.sleep()


if __name__ == "__main__":
    import sys

    try:
        main(rospy.myargv(argv=sys.argv))
    except rospy.exceptions.ROSInterruptException:
        rospy.loginfo("Shutting down CymAckermannServoBoxcar")