"""
APIs creating a ROSLaunch instance that after starting it will launch Gazebo
 with desired number of cars, drones, and specified obstacles
"""

from typing import List, NamedTuple
import xml.etree.ElementTree as ET

from geometry_msgs.msg import Point
import roslaunch
import roslaunch.parent
import roslaunch.rlutil


DeviceInitInfo = NamedTuple(
    "DeviceInitInfo",
    [("bot_name", str), ("bot_type", str), ("position", Point)]
)


def _gen_launch_element_tree(world_name: str,
                             device_list: List[DeviceInitInfo]) -> ET.ElementTree:
    """
    This creates a .launch XML for a given list of device info
    :param device_list: List initial info about a device
    :return: an ElementTree representing desired XML for launch config
    """
    root = ET.Element('launch')
    # include world file
    include = ET.SubElement(
        root, 'include',
        attrib={'file': '$(find cym_gazebo)/launch/cym.template.launch'})
    ET.SubElement(
        include, 'arg',
        attrib={'name': 'world_name', 'value': world_name})
    id_list_str = ' '.join(device.bot_name for device in device_list)
    ET.SubElement(
        include, 'arg',
        attrib={'name': 'id_list', 'value': id_list_str})

    # add devices
    KNOWN_DEVICE = ["f1tenth", "boxcar", "hector_quadrotor"]
    for device in device_list:
        if device.bot_type in KNOWN_DEVICE:
            cym_device_model = device.bot_type
        elif device.bot_type == "CAR":
            cym_device_model = "f1tenth"
        elif device.bot_type == "QUAD":
            cym_device_model = "hector_quadrotor"
        else:
            raise ValueError("Unknown bot_type: " + device.bot_type)

        include = ET.SubElement(
            root, 'include',
            attrib={'file': '$(find cym_device)/launch/spawn_cym_device.launch'})
        ET.SubElement(
            include, 'arg',
            attrib={'name': 'name', 'value': device.bot_name})
        ET.SubElement(
            include, 'arg',
            attrib={'name': 'device_model', 'value': cym_device_model})
        ET.SubElement(include, 'arg', attrib={'name': 'x', 'value': str(device.position.x)})
        ET.SubElement(include, 'arg', attrib={'name': 'y', 'value': str(device.position.y)})
        ET.SubElement(include, 'arg', attrib={'name': 'z', 'value': str(device.position.z)})

    return ET.ElementTree(root)


class _MyROSLaunch(roslaunch.scriptapi.ROSLaunch):
    def __init__(self, launch_xml: ET.ElementTree):
        """
        Create a ROSLaunch instance from a given ElementTree.
            This overrides the original __init__ to avoid waiting for ROS master
        :param launch_xml: Launch xml tree
        :return: ROSLaunch instance load from the launch file
        """
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch_strs = [ET.tostring(launch_xml.getroot(), encoding='unicode')]
        self.parent = roslaunch.parent.ROSLaunchParent(
            uuid,
            [],
            is_core=False,  # is_core=True: throw error if ROS master is already running in background
            roslaunch_strs=launch_strs)
        self.started = False


def create_roslaunch_instance(world_name: str, device_list: List[DeviceInitInfo]) -> roslaunch.scriptapi.ROSLaunch:
    xml = _gen_launch_element_tree(world_name, device_list)
    return _MyROSLaunch(xml)
