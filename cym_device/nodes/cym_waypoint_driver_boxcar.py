#!/usr/bin/env python3

from copy import deepcopy
import math
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped, Quaternion
from std_msgs.msg import Int32
from std_msgs.msg import String


import rospy

from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Float64




class __DeviceState(object):

    HALF_CYCLE = 5.0  # sec
    ABS_VEL = 2.0
    ABS_ANG_VEL = 2.0 * math.pi / 10

    def __init__(self, init_pose: PoseStamped, init_twist: TwistStamped, tracker_id: String):
        assert init_pose
        assert init_twist

        self._pose = init_pose
        self._twist = init_twist
        self._cycle_begin = rospy.get_time()

        self.state = ModelState()

        self.tracker_id = tracker_id




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



    def set_state(self, msg: AckermannDriveStamped) -> ModelState:

        modelInfo = rospy.ServiceProxy('/gazebo/get_model_state',GetModelState)
        currState = modelInfo(model_name=self.tracker_id)
        vel = msg.drive.speed
        steering_angle = msg.drive.steering_angle


        quaternion = (currState.pose.orientation.x,
                      currState.pose.orientation.y,
                      currState.pose.orientation.z,
                      currState.pose.orientation.w)

        r = R.from_quat(quaternion)
        euler = r.as_euler('zyx', degrees=True)
        #euler2 = euler_from_quaternion(quaternion) #(roll,pitch,yaw)


        self.state = ModelState()
        self.state.model_name = self.tracker_id
        self.state.twist.linear.x = vel * math.cos(math.radians(euler[0]))
        self.state.twist.linear.y = vel * math.sin(math.radians(euler[0]))
        self.state.twist.angular.z = vel * (math.tan(steering_angle)/0.5)

        #rospy.loginfo('[%f, %f, %f, %f]',math.radians(euler[0]), math.radians(euler[1]), math.radians(euler[2]), euler2[0])




def main(argv) -> None:
    """
     Main entry point
        This design of putting all pub/sub in main function is intentional
        since we want to avoid pub/sub being instantiated more than once.
    :param argv:
    """
    tracker_id = argv[1]  # TODO Read from parameter server instead
    #tracker_id = 'car1'

    rospy.init_node('waypoint_node')
    # Wait for positioning system to start
    # FIXME Read topic names from parameter server
    pose_topic_name = "/vrpn_client_node/" + tracker_id + "/pose"
    twist_topic_name = "/vrpn_client_node/" + tracker_id + "/twist"
    ackermann_topic_name = "/ackermann_client/" + tracker_id

    init_pose = rospy.wait_for_message(pose_topic_name, PoseStamped)
    init_twist = rospy.wait_for_message(twist_topic_name, TwistStamped)

    ds = __DeviceState(init_pose, init_twist, tracker_id)
    # For positioning
    _ = rospy.Subscriber(pose_topic_name, PoseStamped, ds.store_pose)
    _ = rospy.Subscriber(twist_topic_name, TwistStamped, ds.store_twist)
    # For ackermann
    _ = rospy.Subscriber(ackermann_topic_name, AckermannDriveStamped, ds.set_state)

    modelInfo = rospy.ServiceProxy('/gazebo/get_model_state',GetModelState)


    # For driving the simulated drone
    pub_model_state = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=10)  # FIXME how to decide queue_size

    def shutdown() -> None:  # TODO Better place for this code
        """Stop the drone when this ROS node shuts down"""
        state = ModelState()
        state.model_name = 'car1'
        pub_model_state.publish(state)  # Default Twist will stop the drone
        # TODO Safe landing
        rospy.loginfo("Stop the drone")

    rospy.on_shutdown(shutdown)

    rate = rospy.Rate(100)  # 100 Hz
    while not rospy.is_shutdown():
        state = modelInfo(model_name=tracker_id)

        newState = ModelState()
        newState.model_name = tracker_id
        newState.pose.position.x = state.pose.position.x
        newState.pose.position.y = state.pose.position.y
        newState.pose.orientation.x = state.pose.orientation.x
        newState.pose.orientation.y = state.pose.orientation.y
        newState.pose.orientation.z = state.pose.orientation.z
        newState.pose.orientation.w = state.pose.orientation.w
        newState.twist.linear.x = ds.state.twist.linear.x
        newState.twist.linear.y = ds.state.twist.linear.y
        newState.twist.angular.z = ds.state.twist.angular.z


        pub_model_state.publish(newState)
        rate.sleep()


if __name__ == "__main__":
    import sys

    try:
        main(rospy.myargv(argv=sys.argv))
    except rospy.exceptions.ROSInterruptException:
        rospy.loginfo("Shutting down CymWaypointDriveBoxcar")
