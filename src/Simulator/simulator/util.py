import random, sys, os, _thread, subprocess
import xml.etree.ElementTree as ET
from pathlib import Path
 
# -------------------------------------------------------------------------------------------------------------------
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
            if model == 'car': pose = (random.uniform(-20, 20), random.uniform(-20, 20), 0)
            elif model == 'drone': pose = (random.uniform(-20, 20), random.uniform(-20, 20), random.uniform(1, 10))
            goal_poses.append(pose)
    return goal_poses


# -------------------------------------------------------------------------------------------------------------------


def launch(models, loc):
    '''
    This script creates a XML like .launch file inside catkin_ws3, and it will be launched
    :param loc: dictionary of initial locations of cars and drones
    :param models: a dictionary indicating number of cars and drones
    :return:
    '''
    modelPath = str(Path.home()) + "/catkin_ws3/src/cyphyhouse/launch/cyphyhouse.launch"
    tree = ET.parse(modelPath)
    root = tree.getroot()
    root.clear()

    # include world file
    attrib = {'file': '$(find gazebo_ros)/launch/empty_world.launch'}
    include = ET.SubElement(root, 'include', attrib)
    attrib = {'name': 'world_name', 'value': '$(find cyphyhouse)/worlds/cyphyhouse.world'}
    arg = ET.SubElement(include, 'arg', attrib)

    # add drones
    for i in range(models['drone']):
        x, y, z = loc['drone'][i]
        attrib = {'ns': 'drone'+str(i+1)}
        group = ET.SubElement(root, 'group', attrib)
        attrib = {'file': '$(find hector_quadrotor_gazebo)/launch/spawn_quadrotor.launch'}
        include = ET.SubElement(group, 'include', attrib)
        attrib = {'name': 'name', 'value': 'drone'+str(i+1)}
        arg = ET.SubElement(include, 'arg', attrib)
        attrib = {'name': 'model', 'value': '$(find hector_quadrotor_description)/urdf/quadrotor_hokuyo_utm30lx.gazebo.xacro'}
        arg = ET.SubElement(include, 'arg', attrib)
        attrib = {'name': 'controllers', 'value': 'controller/attitude controller/velocity controller/position'}
        arg = ET.SubElement(include, 'arg', attrib)
        attrib = {'name': 'x', 'value': str(x)}
        arg = ET.SubElement(include, 'arg', attrib)
        attrib = {'name': 'y', 'value': str(y)}
        arg = ET.SubElement(include, 'arg', attrib)
        attrib = {'name': 'z', 'value': str(z)}
        arg = ET.SubElement(include, 'arg', attrib)

    for i in range(models['car']):
        x, y, z = loc['car'][i]
        attrib = {'ns': 'car' + str(i + 1)}
        group = ET.SubElement(root, 'group', attrib)
        attrib = {'file': '$(find f1tenth)/launch/car.launch'}
        include = ET.SubElement(group, 'include', attrib)
        attrib = {'name': 'name', 'value': 'car' + str(i + 1)}
        arg = ET.SubElement(include, 'arg', attrib)
        attrib = {'name': 'x', 'value': str(x)}
        arg = ET.SubElement(include, 'arg', attrib)
        attrib = {'name': 'y', 'value': str(y)}
        arg = ET.SubElement(include, 'arg', attrib)
        attrib = {'name': 'z', 'value': str(z)}
        arg = ET.SubElement(include, 'arg', attrib)

        attrib = {'file': '$(find f1tenth)/launch/car_control.launch'}
        include = ET.SubElement(root, 'include', attrib)
        attrib = {'name': 'name', 'value': 'car' + str(i + 1)}
        arg = ET.SubElement(include, 'arg', attrib)
        

    car_controller_launch_file_generator(models['car'])
    attrib = {'file': '$(find cyphy_car_mpc)/launch/wp.launch'}
    include = ET.SubElement(root, 'include', attrib)
    


    tree.write(modelPath)
    print("roslaunch")
    # os.system("roslaunch drone drone.launch")
    # _thread.start_new_thread(os.system, ("roslaunch drone drone.launch",))
    proc = subprocess.Popen(['roslaunch', 'cyphyhouse', 'cyphyhouse.launch'])

    return proc

def car_controller_launch_file_generator(num_car):
    content = '''
    <!-- -*- mode: XML -*- -->
    <launch>
    '''
    for i in range(num_car):
        content += "\t<node name=\"car"+ str(i+1) +"\" pkg=\"cyphy_car_mpc\" type=\"mpc_wp_node\" output=\"screen\" > </node>\n"

    content += "</launch>"

    path = str(Path.home()) + "/catkin_ws3/src/cyphy_car_mpc/launch/wp.launch"
    f = open(path, "w")
    f.write(content)
    f.close()

def sim_launch(models, loc):
    '''
    This function calls launch method to create launch file and use roslaunch to initiate simulator
    :param models: A dictionary that contains number of drones and number cars
    :param loc: Initial location of models
    :return: The initiated Gazebo-ROS process
    '''
    # import launch
    ros_proc = launch(models, loc)
    print("============= Simulator starts successful ================")
    # time.sleep(max(num * 6, 10))
    return ros_proc
    
if __name__ == '__main__':
    pass