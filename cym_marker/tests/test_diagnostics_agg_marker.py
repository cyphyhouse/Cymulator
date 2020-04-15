#!/usr/bin/env python3
import yaml

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import rospy


def factory_msg(i: int) -> DiagnosticArray:
    lst = []
    for j in range(5):
        p0 = [j] + [0.5*(j+1)]*3
        p1 = [j+1] + [0.5*(j+3)]*3
        lst.append(p0)
        lst.append(p1)

    yaml_str = str(yaml.safe_dump(lst))

    stat = DiagnosticStatus(
        name="reachset",
        hardware_id="drone0",
        values=[
            KeyValue(key="format", value="yaml"),
            KeyValue(key="data", value=yaml_str)])
    return DiagnosticArray(status=[stat])


def main(argv):
    rospy.init_node('test_diagnostics_agg')

    pub_marker = rospy.Publisher("/diagnostics_agg", DiagnosticArray,
                                 queue_size=10)

    msg_list = []
    for i in range(1):
        msg_list.append(factory_msg(i))

    rate = rospy.Rate(0.25)
    for msg in msg_list:
        rate.sleep()
        pub_marker.publish(msg)


if __name__ == "__main__":
    import sys

    try:
        main(rospy.myargv(argv=sys.argv))
    except rospy.exceptions.ROSInterruptException:
        rospy.loginfo("Shutting down TestCymMarker")
