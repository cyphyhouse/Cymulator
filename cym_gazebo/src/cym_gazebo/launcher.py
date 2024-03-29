"""
APIs creating a ROSLaunch XML tree that after starting it will launch Gazebo
 with desired number of cars, drones, and specified obstacles
"""

from typing import Any, Dict, NamedTuple, Sequence
import xml.etree.ElementTree as ET

from geometry_msgs.msg import Point
import rospy


DeviceInitInfo = NamedTuple(
    "DeviceInitInfo",
    [("bot_name", str), ("bot_type", str), ("position", Point), ("yaw", float)]
)


def gen_launch_element_tree(cfg) -> ET.ElementTree:
    """
    This creates a .launch XML for a given list of device info
    :param cfg: Configuration specifying the devices and world.
    :return: an ElementTree representing desired XML for launch config
    """
    if isinstance(cfg, list):
        world_name = "irl_arena.world"
        cfg_devices = cfg  # type: Sequence[Dict[str, Any]]
    elif isinstance(cfg, dict):
        world_name = cfg['world_name']
        cfg_devices = cfg['devices']
    else:
        raise ValueError("Unexpected value in YAML file")

    device_list = [
        DeviceInitInfo(
            bot_name=device["bot_name"],
            bot_type=device["bot_type"],
            position=Point(*device["init_pos"]),
            yaw=device.get("init_yaw", 0.0))
        for device in cfg_devices
    ]

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
    KNOWN_DEVICES = ["f1tenth", "boxcar", "hector_quadrotor", "rosplane"]
    for device in device_list:
        if device.bot_type.lower() in KNOWN_DEVICES:
            cym_device_model = device.bot_type.lower()
        elif device.bot_type.upper() in ["CAR", "RACECAR"]:
            cym_device_model = "f1tenth"
        elif device.bot_type.upper() in ["QUAD", "DRONE"]:
            cym_device_model = "hector_quadrotor"
        elif device.bot_type.upper() in ["PLANE", "FIXEDWING"]:
            cym_device_model = "rosplane"
        else:
            raise ValueError("Unknown bot_type: " + device.bot_type)
        assert cym_device_model in KNOWN_DEVICES

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

        if device.yaw != 0.0 and cym_device_model == "hector_quadrotor":
            rospy.logwarn("Warning: Initial yaw heading %f of %s will be ignored by Hector Quadrotor model." % (device.yaw, device.bot_name))
        ET.SubElement(include, 'arg', attrib={'name': 'yaw', 'value': str(device.yaw)})

    return ET.ElementTree(root)
