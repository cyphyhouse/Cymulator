import xml.etree.ElementTree as ET
from pathlib import Path
import os
import _thread
import subprocess


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

    tree.write(modelPath)
    print("roslaunch")
    # os.system("roslaunch drone drone.launch")
    # _thread.start_new_thread(os.system, ("roslaunch drone drone.launch",))
    proc = subprocess.Popen(['roslaunch', 'cyphyhouse', 'cyphyhouse.launch'])

    return proc


if __name__ == '__main__':
    loc = {'drone': [[0, 0, 0.3], [0, 5, 0.3]], 'car': [[5, 5, 0.3], [5, 0, 0.3]]}
    models = {'drone': 2, 'car': 2}
    launch(loc, models)