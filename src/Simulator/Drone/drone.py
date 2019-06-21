#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2, sqrt
from hector_uav_msgs.srv import EnableMotors


class Drone():
    def __init__(self, number):
        # Drone's position and orientation inforamtion
        self._x = 0.0
        self._y = 0.0
        self._z = 0.0
        self._theta = 0.0
        self.goal = Point()

        # Set up subscriber and publisher
        my_number = "/drone" + str(number)
        self.sub = rospy.Subscriber(my_number+"/ground_truth/state", Odometry, self.newPos)
        self.pub = rospy.Publisher(my_number+"/cmd_vel", Twist, queue_size=10)
        
        # Enable motors using ROS service
        rospy.wait_for_service(my_number+'/enable_motors')
        try:
            # Set 'enable_motors' to be True
            enable = rospy.ServiceProxy(my_number+'/enable_motors', EnableMotors)
            resp = enable(True)
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed with %s", e)

    def newPos(self, msg):
        self._x = msg.pose.pose.position.x 
        self._y = msg.pose.pose.position.y 
        self._z = msg.pose.pose.position.z 

        quat = msg.pose.pose.orientation
        (_, _, self._theta) = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])

