import rospy
from geometry_msgs.msg import Pose, Vector3

from cym_marker.msg import Marker, Material, Script


def factory_script(i: int) -> Script:
    PREDEFINED_SCRIPT = [
       "Gazebo/RedTransparent",
       "Gazebo/GreenTransparent",
       "Gazebo/BlueTransparent",
       "Gazebo/DarkMagentaTransparent",
       "Gazebo/GreyTransparent",
       "Gazebo/BlackTransparent",
       "Gazebo/YellowTransparent",
    ]
    return Script(name=PREDEFINED_SCRIPT[i % len(PREDEFINED_SCRIPT)])


def factory_pose(i: int) -> Pose:
    pose = Pose()
    pose.position.x = i - 5
    pose.position.y = i - 5
    pose.position.z = i
    pose.orientation.x = 0
    pose.orientation.y = 0
    pose.orientation.z = 0
    pose.orientation.w = 1
    return pose


def factory_scale(i: int) -> Vector3:
    return Vector3(x=1.0, y=1.0, z=1.0)


def factory_marker(i: int, action = Marker.ADD_MODIFY) -> Marker:
    SIMPLE_TYPE = [
        Marker.BOX,
        Marker.CYLINDER,
        Marker.SPHERE,
        Marker.TEXT
    ]

    mat = Material(script=factory_script(i))

    marker = Marker()
    marker.header.frame_id = "world"
    marker.action = action
    marker.id = i
    if action != Marker.ADD_MODIFY:
        return marker

    marker.type = SIMPLE_TYPE[i % len(SIMPLE_TYPE)]
    marker.pose = factory_pose(i)
    marker.scale = factory_scale(i) 
    marker.material = mat

    if marker.type == Marker.TEXT:
        marker.text = "Hello world!"

    return marker


def main(argv):
    rospy.init_node('test_cym_marker')

    pub_marker = rospy.Publisher("/cym_marker", Marker, queue_size=10)

    marker_list = []
    for i in range(0, 10):
        marker_list.append(factory_marker(i))
    for i in range(0, 10):
        marker_list.append(factory_marker(i, Marker.DELETE_MARKER))

    i = 0
    rate = rospy.Rate(0.25)  # ROS will try to finish an iteration in this frequency
    while not rospy.is_shutdown():
        rate.sleep()
        pub_marker.publish(marker_list[i])
        i = (i + 1) % len(marker_list)


if __name__ == "__main__":
    import sys

    try:
        main(rospy.myargv(argv=sys.argv))
    except rospy.exceptions.ROSInterruptException:
        rospy.loginfo("Shutting down TestCymMarker")

