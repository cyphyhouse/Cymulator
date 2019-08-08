#!/usr/bin/env python3

from geometry_msgs.msg import Point

from cym_gazebo import DeviceInitInfo, create_roslaunch_instance

devices = [
    DeviceInitInfo("hotdec_car", "CAR", Point(1, 1, 0.3)),
    DeviceInitInfo("drone1", "QUAD", Point(2, 2, 0.3)),
    DeviceInitInfo("f1car", "CAR", Point(3, 3, 0.3)),
]

launch = create_roslaunch_instance(devices)
try:
    launch.start()
    launch.spin()
finally:
    launch.stop()
