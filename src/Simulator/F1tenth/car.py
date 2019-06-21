#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import Float64
from math import atan2, sqrt

class Car():
    def __init__(self, number):
        # Robot's position and orientation inforamtion
        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0
        self.goal = Point()

        # Set up subscriber and publisher
        my_number = "/car" + str(number)
        self.pub_vel_left_rear_wheel = rospy.Publisher(my_number + '/racecar/left_rear_wheel_velocity_controller/command', Float64, queue_size=1)
        self.pub_vel_right_rear_wheel = rospy.Publisher(my_number + '/racecar/right_rear_wheel_velocity_controller/command', Float64, queue_size=1)
        self.pub_vel_left_front_wheel = rospy.Publisher(my_number + '/racecar/left_front_wheel_velocity_controller/command', Float64, queue_size=1)
        self.pub_vel_right_front_wheel = rospy.Publisher(my_number + '/racecar/right_front_wheel_velocity_controller/command', Float64, queue_size=1)

        self.pub_pos_left_steering_hinge = rospy.Publisher(my_number + '/racecar/left_steering_hinge_position_controller/command', Float64, queue_size=1)
        self.pub_pos_right_steering_hinge = rospy.Publisher(my_number + '/racecar/right_steering_hinge_position_controller/command', Float64, queue_size=1)

        self.sub = rospy.Subscriber(my_number + "/ground_truth/state", Odometry, self.newPos)

    def newPos(self, msg):
        self._x = msg.pose.pose.position.x 
        self._y = msg.pose.pose.position.y

        quat = msg.pose.pose.orientation
        (_, _, self._theta) = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
