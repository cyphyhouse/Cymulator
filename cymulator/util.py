import random
import xml.etree.ElementTree as ET


def parse_init_pose(num, poses):
    '''
    This function parses the initial positions of models; if argument 'poses' is an empty list,
    set models to random positions
    :param num: Number of models
    :param poses: User_input list of initial positions
    :return: init_poses - parsed initial positions
    '''
    init_poses = []
    for pose in poses:
        pose = list((int(p) for p in pose.strip('()').split(',')))
        pose.append(0.3)
        init_poses.append(tuple(pose))
    if len(poses) < num:
        for _ in range(num - len(poses)):
            pose = (random.uniform(-10, 10), random.uniform(-10, 10), 0.3)
            init_poses.append(pose)
    return init_poses


def parse_goal_pose(num, poses, model):
    '''
    This function parses the goal positions of models; if argument 'poses' is empty,
    set models' goals to random positions
    :param num: Number of models
    :param poses: User_input list of goal positions
    :param model: the model type
    :return: goal_poses - parsed goal positions
    '''
    if len(poses) > 0 and type(poses[0]) == list:
        return poses
    goal_poses = []
    for pose in poses:
        pose = list((int(p) for p in pose.strip('()').split(',')))
        if model == 'car': pose.append(0)
        goal_poses.append(tuple(pose))
    if len(poses) < num:
        for _ in range(num - len(poses)):
            if model == 'car':
                pose = (random.uniform(-20, 20), random.uniform(-20, 20), 0)
            elif model == 'drone':
                pose = (random.uniform(-20, 20), random.uniform(-20, 20), random.uniform(1, 10))
            else:
                raise ValueError("Unknown model: " + model)
            goal_poses.append(pose)
    return goal_poses


def gen_launch_element_tree(models, loc) -> ET.ElementTree:
    """
    This script creates a XML like .launch file inside catkin_ws3, and it will be launched
    :param loc: dictionary of initial locations of cars and drones
    :param models: a dictionary indicating number of cars and drones
    :return: an ElementTree representing desired XML for launch config
    """
    root = ET.Element('launch')
    # include world file
    include = ET.SubElement(
        root, 'include',
        attrib={'file': '$(find gazebo_ros)/launch/empty_world.launch'})
    ET.SubElement(
        include, 'arg',
        attrib={'name': 'world_name', 'value': '$(find cyphyhouse)/worlds/cyphyhouse.world'})

    id_str_list = []

    # add drones
    for i in range(models['drone']):
        id_str = "drone" + str(i+1)
        id_str_list.append(id_str)
        include = ET.SubElement(
            root, 'include',
            attrib={'file': '$(find cym_device)/launch/cym_drone.launch'})
        ET.SubElement(
            include, 'arg',
            attrib={'name': 'name', 'value': id_str})
        ET.SubElement(
            include, 'arg',
            attrib={'name': 'pos_tracker', 'value': id_str})  # FIXME distinguish pos_tracker with id_str
        x, y, z = loc['drone'][i]
        ET.SubElement(include, 'arg', attrib={'name': 'x', 'value': str(x)})
        ET.SubElement(include, 'arg', attrib={'name': 'y', 'value': str(y)})
        ET.SubElement(include, 'arg', attrib={'name': 'z', 'value': str(z)})
    # add cars
    # TODO Merge for loops for drone and car to remove duplicate code?
    for i in range(models['car']):
        id_str = "car" + str(i + 1)
        id_str_list.append(id_str)
        include = ET.SubElement(
            root, 'include',
            attrib={'file': '$(find cym_device)/launch/cym_car.launch'})
        ET.SubElement(
            include, 'arg',
            attrib={'name': 'name', 'value': id_str})
        ET.SubElement(
            include, 'arg',
            attrib={'name': 'pos_tracker', 'value': id_str})  # FIXME distinguish pos_tracker with id_str
        x, y, z = loc['car'][i]
        ET.SubElement(include, 'arg', attrib={'name': 'x', 'value': str(x)})
        ET.SubElement(include, 'arg', attrib={'name': 'y', 'value': str(y)})
        ET.SubElement(include, 'arg', attrib={'name': 'z', 'value': str(z)})

    # add VRPN
    ET.SubElement(
        root, 'node',
        attrib={'name': 'vrpn_client_node',
                'pkg': 'cym_device',
                'type':'cym_vrpn.py',
                'args': ' '.join(id_str_list)}  # FIXME Pass pos_trackers as ROS param
    )

    return ET.ElementTree(root)

