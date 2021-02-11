#!/usr/bin/env python3

import roslaunch
import rospy

from cym_gazebo import gen_launch_element_tree

import os
import sys
import tempfile
import yaml


argv = rospy.myargv(argv=sys.argv)
cfg_yml = argv[1]

with open(cfg_yml, 'r') as f:
    cfg = yaml.safe_load(f)
    launch_xml_tree = gen_launch_element_tree(cfg)

# Create a temporary launch file and launch using roslaunch
_, path = tempfile.mkstemp()
try:
    launch_xml_tree.write(path)

    roslaunch.main(argv=[argv[0], path])
finally:
    os.remove(path)
