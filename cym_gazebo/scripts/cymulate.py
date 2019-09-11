#!/usr/bin/env python3

from geometry_msgs.msg import Point
import rospy

from cym_gazebo import DeviceInitInfo, create_roslaunch_instance

import sys
import yaml


argv = rospy.myargv(argv=sys.argv)
cfg_yml = argv[1]

with open(cfg_yml, 'r') as f:
    cfg = yaml.safe_load(f)
    devices = [
        DeviceInitInfo(device["bot_name"], device["bot_type"], Point(*device["init_pos"]))
        for device in cfg
    ]


launch = create_roslaunch_instance(devices)
try:
    launch.start()
    launch.spin()
finally:
    launch.stop()

