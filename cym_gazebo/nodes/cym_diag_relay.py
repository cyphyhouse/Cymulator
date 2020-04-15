#!/usr/bin/env python3

"""
This is an example publisher that extracts and relays all relevant ROS topics
to diagnostic messages from Gazebo simulations.
Note that this node relays messages for all drones for simplicity.
This does not make our experiments easier because original topics may still be
published in different rates.
In practice, each vehicle should publish its own dignostic messages.
"""

import base64
from io import BytesIO
import sys
from typing import List

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from gazebo_msgs.msg import ModelStates
from genpy import Message
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import rospy


def to_serialized_str(msg: Message) -> str:
    buff = BytesIO()
    msg.serialize(buff)
    data = buff.getvalue()
    return base64.encodebytes(data).decode('ascii')


class RelayDiagnostic:
    def __init__(self, pub_diag: rospy.Publisher, device_list: List[str]):
        self.__pub_diag = pub_diag
        self.__device_set = set(device_list)

    def relay_state(self, msg: ModelStates):
        # NOTE Agent to id may change because models may be added or removed
        device_to_id = {msg.name[i]: i
                        for i in range(len(msg.name))
                        if msg.name[i] in self.__device_set}

        diag_list = []
        for device in self.__device_set & set(device_to_id.keys()):
            pose_str = to_serialized_str(msg.pose[device_to_id[device]])
            twist_str = to_serialized_str(msg.twist[device_to_id[device]])

            diag_status = DiagnosticStatus(
                level=DiagnosticStatus.OK,
                name="state",
                message="Update state values",
                hardware_id=device,
                values=[
                    KeyValue(key="pose", value=pose_str),
                    KeyValue(key="twist", value=twist_str),
                ]
            )
            diag_list.append(diag_status)

        if not diag_list:
            return  # Nothing to publish
        diag_msg = DiagnosticArray(
            header=Header(stamp=rospy.Time.now()),
            status=diag_list
        )
        self.__pub_diag.publish(diag_msg)

    def relay_input(self, device: str) -> None:
        def callback(msg: PoseStamped):
            # TODO
            position_str = to_serialized_str(msg.pose.position)
            diag_status = DiagnosticStatus(
                level=DiagnosticStatus.OK,
                name="input",
                message="Update input values",
                hardware_id=device,
                values=[
                    KeyValue(key="position", value=position_str),
                ]
            )

            diag_msg = DiagnosticArray(
                header=msg.header,
                status=[diag_status]
            )
            self.__pub_diag.publish(diag_msg)

        return callback


def main(argv):

    device_list = argv[1:]  # TODO get devices from parameters
    if not device_list:
        rospy.logwarn("No device id is specified for diagnostics. "
                      "Shutting down.")
        return

    rospy.init_node('diagnostic_publisher', anonymous=True)
    pub_diag = rospy.Publisher("/diagnostics", DiagnosticArray, queue_size=10)

    relay = RelayDiagnostic(pub_diag, device_list)
    _ = rospy.Subscriber("/gazebo/model_states", ModelStates,
                         relay.relay_state)

    for device in device_list:
        callback = relay.relay_input(device)
        _ = rospy.Subscriber(device + "/command/pose",
                             PoseStamped, callback)

    rospy.spin()


if __name__ == "__main__":
    try:
        main(rospy.myargv(argv=sys.argv))
    except rospy.exceptions.ROSInterruptException:
        rospy.loginfo("Shutting down ")
