#!/usr/bin/env python3
"""ROS node mimicking VRPN server to publish positions of all devices"""

from typing import NamedTuple, Tuple, Optional

from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import PoseStamped, TwistStamped
import rospy


VRPNTopicPublishers = NamedTuple(
    'VRPNTopicPublishers',
    [('pose', rospy.Publisher),
     ('twist', rospy.Publisher)]
)


def get_position(model_name: str) -> Optional[Tuple[PoseStamped, TwistStamped]]:
    service_name = '/gazebo/get_model_state'
    rospy.wait_for_service(service_name)
    get_model_state = rospy.ServiceProxy(service_name, GetModelState)

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


def main(argv):
    """ Main entry point
        This design of putting all pub/sub in main function is intentional
        since we want to avoid pub/sub being instantiated more than once.
    """
    # TODO Read from parameter server instead
    tracker_list = argv[1:4]
    rospy.init_node('vrpn_client_node')
    rate = rospy.Rate(10)  # 10hz TODO Pass sleep rate as a parameter?
    # Queue size is 1 since old positions are not needed
    pub = {}
    for tracker_id in tracker_list:
        pub[tracker_id] = VRPNTopicPublishers(
            pose=rospy.Publisher("~" + tracker_id + "/pose", PoseStamped, queue_size=1),
            twist=rospy.Publisher("~" + tracker_id + "/twist", TwistStamped, queue_size=1)
        )
    # XXX Should we wait until all models are created?
    # Gazebo throws errors because this may try to get model states even before the model is spawned
    # On the otherhand, if the model is never spawned then waiting can be problematic.
    while not rospy.is_shutdown():
        for tracker_id in tracker_list:
            position = get_position(tracker_id)
            if not position:
                # FIXME What to do if we cannot get the state of a model
                pass  # Not publishing new positions
            else:
                pose, twist = position
                pub[tracker_id].pose.publish(pose)
                pub[tracker_id].twist.publish(twist)
        rate.sleep()


if __name__ == "__main__":
    import sys
    main(sys.argv)
