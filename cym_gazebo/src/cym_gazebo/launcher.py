"""
APIs creating a ROSLaunch XML tree that after starting it will launch Gazebo
 with desired number of cars, drones, and specified obstacles
"""

from typing import List, NamedTuple
import xml.etree.ElementTree as ET

from geometry_msgs.msg import Point


DeviceInitInfo = NamedTuple(
    "DeviceInitInfo",
    [("bot_name", str), ("bot_type", str), ("position", Point)]
)


def gen_launch_element_tree(cfg) -> ET.ElementTree:
    """
    This creates a .launch XML for a given list of device info
    :param cfg: Configuration specifying the devices and world.
    :return: an ElementTree representing desired XML for launch config
    """
    if isinstance(cfg, list):
        world_name = "irl_arena.world"
        cfg_devices = cfg
    elif isinstance(cfg, dict):
        world_name = cfg['world_name']
        cfg_devices = cfg['devices']
    else:
        raise ValueError("Unexpected value in YAML file")

    device_list = [
        DeviceInitInfo(device["bot_name"], device["bot_type"], Point(*device["init_pos"]))
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
