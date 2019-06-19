import xml.etree.ElementTree as ET
from pathlib import Path
import os


def launch(num, loc):
    modelPath = str(Path.home()) + "/catkin_ws3/src/drone/launch/drone.launch"
    tree = ET.parse(modelPath)
    root = tree.getroot()
    root.clear()

    # include world file
    attrib = {'file': '$(find gazebo_ros)/launch/empty_world.launch'}
    include = ET.SubElement(root, 'include', attrib)
    attrib = {'name': 'world_name', 'value': '$(find drone)/worlds/drone.world'}
    arg = ET.SubElement(include, 'arg', attrib)

    # add drones
    for i in range(num):
        x, y, z = loc[i]
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

    tree.write(modelPath)

    os.system("roslaunch drone drone.launch")


if __name__ == '__main__':
    launch(3,[[0, 0, 0.3], [0, 5, 0.3], [5, 5, 0.3], [5, 0, 0.3]])