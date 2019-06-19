import xml.etree.ElementTree as ET
from pathlib import Path
import os


def launch(num, loc):
    modelPath = str(Path.home()) + "/catkin_ws3/src/f1tenth/launch/f1tenth.launch"
    tree = ET.parse(modelPath)
    root = tree.getroot()
    root.clear()

    # include world file
    attrib = {'file': '$(find gazebo_ros)/launch/empty_world.launch'}
    include = ET.SubElement(root, 'include', attrib)
    attrib = {'name': 'world_name', 'value': '$(find f1tenth)/worlds/f1tenth.world'}
    arg = ET.SubElement(include, 'arg', attrib)

    # add drones
    for i in range(num):
        x, y, z = loc[i]
        attrib = {'ns': 'car'+str(i+1)}
        group = ET.SubElement(root, 'group', attrib)
        attrib = {'file': '$(find f1tenth)/launch/car.launch'}
        include = ET.SubElement(group, 'include', attrib)
        attrib = {'name': 'name', 'value': 'car'+str(i+1)}
        arg = ET.SubElement(include, 'arg', attrib)
        attrib = {'name': 'x', 'value': str(x)}
        arg = ET.SubElement(include, 'arg', attrib)
        attrib = {'name': 'y', 'value': str(y)}
        arg = ET.SubElement(include, 'arg', attrib)
        attrib = {'name': 'z', 'value': str(z)}
        arg = ET.SubElement(include, 'arg', attrib)

        attrib = {'file': '$(find f1tenth)/launch/car_control.launch'}
        include = ET.SubElement(root, 'include', attrib)
        attrib = {'name': 'name', 'value': 'car'+str(i+1)}
        arg = ET.SubElement(include, 'arg', attrib)

    attrib = {'file': '$(find cyphy_car)/launch/wp.launch'}
    include = ET.SubElement(root, 'include', attrib)

    tree.write(modelPath)

    os.system("roslaunch f1tenth f1tenth.launch")


if __name__ == '__main__':
    launch(4, [[0, 0, 0.3], [0, 5, 0.3], [5, 5, 0.3], [5, 0, 0.3]])