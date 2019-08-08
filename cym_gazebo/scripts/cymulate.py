#!/usr/bin/env python3

from geometry_msgs.msg import Point

from cym_gazebo import DeviceInitInfo, gen_launch_element_tree, MyROSLaunch

devices = [
    DeviceInitInfo("hotdec_car", "CAR", Point(1, 1, 0.3)),
    DeviceInitInfo("drone1", "QUAD", Point(2, 2, 0.3)),
    DeviceInitInfo("f1car", "CAR", Point(3, 3, 0.3)),
]

xml = gen_launch_element_tree(devices)
launch = MyROSLaunch(xml)
try:
    launch.start()
    launch.spin()
finally:
    launch.stop()
