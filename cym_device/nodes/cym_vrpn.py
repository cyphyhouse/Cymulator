#!/usr/bin/env python3
"""ROS node mimicking VRPN server to publish positions of all devices"""

from typing import NamedTuple, Tuple, Optional, List

from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import PoseStamped, TwistStamped
import rospy


VRPNTopicPublishers = NamedTuple(
    'VRPNTopicPublishers',
    [('pose', rospy.Publisher),
     ('twist', rospy.Publisher)]
)


def get_position(model_name: str) -> Optional[Tuple[PoseStamped, TwistStamped]]:
    """
    Get the position of a given model from Gazebo
    :param model_name: Name of the Gazebo model
    :return: Position in the types used by VRPN
    """
    service_name = '/gazebo/get_model_state'
    rospy.wait_for_service(service_name)
    get_model_state = rospy.ServiceProxy(service_name, GetModelState)

    # TODO Handle service exception. E.g., Gazebo may be shutdown before this node so no service.
    model_state = get_model_state(model_name, "world")
    if not model_state.success:
        return None
    # else:
    pose_stamped = PoseStamped(
        header=model_state.header,
        pose=model_state.pose
    )
    twist_stamped = TwistStamped(
        header=model_state.header,
        twist=model_state.twist
    )
    return pose_stamped, twist_stamped


def main(argv: List[str]) -> None:
    """
     Main entry point
        This design of putting all pub/sub in main function is intentional
        since we want to avoid pub/sub being instantiated more than once.
    :param argv:
    """
    # TODO Read from parameter server instead
    tracker_list = argv[1:]
    rospy.init_node('vrpn_client_node')
    # Queue size is 1 since old positions are not needed
    pub = {}
    for tracker_id in tracker_list:
        pub[tracker_id] = VRPNTopicPublishers(
            pose=rospy.Publisher("~" + tracker_id + "/pose", PoseStamped, queue_size=1),
            twist=rospy.Publisher("~" + tracker_id + "/twist", TwistStamped, queue_size=1)
        )
    # TODO Check if Gazebo finished creating all models to avoid the error messages
    #  "GetModelState: model [model_name] does not exist"
    # XXX Should we wait until all models are created?
    #  Gazebo throws errors because this may try to get model states even before the model is spawned
    #  On the other hand, if the model is never spawned or deleted then waiting can be problematic.
    rate = rospy.Rate(100)  # 10hz TODO Pass sleep rate as a parameter?
    while not rospy.is_shutdown():
        rate.sleep()
        for tracker_id in tracker_list:
            position = get_position(tracker_id)
            if not position:
                # FIXME What to do if we cannot get the state of a model
                pass  # Not publishing new positions
            else:
                pose, twist = position
                pub[tracker_id].pose.publish(pose)
                pub[tracker_id].twist.publish(twist)


if __name__ == "__main__":
    import sys

    try:
        main(rospy.myargv(argv=sys.argv))
    except rospy.exceptions.ROSInterruptException:
        rospy.loginfo("Shutting down CymVRPN")
