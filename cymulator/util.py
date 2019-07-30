import random
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path


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


def launch(models, loc):
    """
    This script creates a XML like .launch file inside catkin_ws3, and it will be launched
    :param loc: dictionary of initial locations of cars and drones
    :param models: a dictionary indicating number of cars and drones
    :return:
    """
    root = ET.Element('launch')
    # include world file
    include = ET.SubElement(
        root, 'include',
        attrib={'file': '$(find gazebo_ros)/launch/empty_world.launch'})
    arg = ET.SubElement(
        include, 'arg',
        attrib={'name': 'world_name', 'value': '$(find cyphyhouse)/worlds/cyphyhouse.world'})

    # add drones
    for i in range(models['drone']):
        id_str = "drone" + str(i+1)
        group = ET.SubElement(
            root, 'group',
            attrib={'ns': id_str})
        include = ET.SubElement(
            group, 'include',
            attrib={'file': '$(find hector_quadrotor_gazebo)/launch/spawn_quadrotor.launch'})
        ET.SubElement(
            include, 'arg',
            attrib={'name': 'name', 'value': id_str})
        ET.SubElement(
            include, 'arg',
            attrib={'name': 'model',
                    'value': '$(find hector_quadrotor_description)/urdf/quadrotor_hokuyo_utm30lx.gazebo.xacro'})
        ET.SubElement(
            include, 'arg',
            attrib={'name': 'controllers', 'value': 'controller/attitude controller/velocity controller/position'})

        x, y, z = loc['drone'][i]
        ET.SubElement(include, 'arg', attrib={'name': 'x', 'value': str(x)})
        ET.SubElement(include, 'arg', attrib={'name': 'y', 'value': str(y)})
        ET.SubElement(include, 'arg', attrib={'name': 'z', 'value': str(z)})

    for i in range(models['car']):
        id_str = "car" + str(i + 1)
        group = ET.SubElement(root, 'group', attrib={'ns': id_str})
        include = ET.SubElement(group, 'include', attrib={'file': '$(find f1tenth)/launch/car.launch'})
        ET.SubElement(include, 'arg', attrib={'name': 'name', 'value': id_str})

        x, y, z = loc['car'][i]
        ET.SubElement(include, 'arg', attrib={'name': 'x', 'value': str(x)})
        ET.SubElement(include, 'arg', attrib={'name': 'y', 'value': str(y)})
        ET.SubElement(include, 'arg', attrib={'name': 'z', 'value': str(z)})

        include = ET.SubElement(
            root, 'include',
            attrib={'file': '$(find f1tenth)/launch/car_control.launch'})
        ET.SubElement(include, 'arg', attrib={'name': 'name', 'value': id_str})

        include = ET.SubElement(
            root, 'include',
            attrib={'file': '$(find cyphy_car_mpc)/launch/waypoint_node.launch'})
        ET.SubElement(include, 'arg', attrib={'name': 'name', 'value': id_str})

    tree = ET.ElementTree(root)
    model_path = str(Path.home()) + "/catkin_ws3/src/cyphyhouse/launch/cyphyhouse.launch"
    tree.write(model_path)
    print("roslaunch")
    proc = subprocess.Popen(['roslaunch', 'cyphyhouse', 'cyphyhouse.launch'])

    return proc
