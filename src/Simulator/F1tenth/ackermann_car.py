#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PointStamped, Twist, Pose
from std_msgs.msg import Float64
from math import atan2, sqrt
from ackermann_msgs.msg import AckermannDriveStamped
import sys


# TODO: this file needs to be changed corresponding to the new MPC controller
class AckermannCar:
    def __init__(self, num):
        # Set up subscriber and publisher
        my_number = 'car' + str(num)
        self.pub_vel_left_rear_wheel = rospy.Publisher(my_number + '/racecar/left_rear_wheel_velocity_controller/command', Float64, queue_size=1)
        self.pub_vel_right_rear_wheel = rospy.Publisher(my_number + '/racecar/right_rear_wheel_velocity_controller/command', Float64, queue_size=1)
        self.pub_vel_left_front_wheel = rospy.Publisher(my_number + '/racecar/left_front_wheel_velocity_controller/command', Float64, queue_size=1)
        self.pub_vel_right_front_wheel = rospy.Publisher(my_number + '/racecar/right_front_wheel_velocity_controller/command', Float64, queue_size=1)

        self.pub_pos_left_steering_hinge = rospy.Publisher(my_number + '/racecar/left_steering_hinge_position_controller/command', Float64, queue_size=1)
        self.pub_pos_right_steering_hinge = rospy.Publisher(my_number + '/racecar/right_steering_hinge_position_controller/command', Float64, queue_size=1)

        rospy.loginfo("Subscribe ackermann message!")
        self.ackermann = rospy.Subscriber(my_number + "/ackermann_cmd", AckermannDriveStamped, self.set_throttle)

    def set_throttle(self, data):
        # if not self.goal_sent:
        # self.goal_sent = True
        # rospy.loginfo("Send goal!")
        # pub_goal = rospy.Publisher("/car_goal/goal", PointStamped, queue_size=1)
        # goal = PointStamped()
        # goal.point.x = 10
        # goal.point.y = 20
        # goal.header.frame_id = "1"
        # pub_goal.publish(goal)

        rospy.loginfo("Command received!")
        throttle = data.drive.speed/0.1
        steer = data.drive.steering_angle
        rospy.loginfo("Set throttle: %f, set steer: %f. ", throttle, steer)
        self.pub_pos_right_steering_hinge.publish(steer)
        self.pub_pos_left_steering_hinge.publish(steer)
        self.pub_vel_left_rear_wheel.publish(throttle)
        self.pub_vel_right_rear_wheel.publish(throttle)
        self.pub_vel_left_front_wheel.publish(throttle)
        self.pub_vel_right_front_wheel.publish(throttle)


class Listen:
    def __init__(self, num):
        self.numberOfCars = num
        self.cars = []

        # Setup drones
        for i in range(self.numberOfCars):
            self.cars.append(AckermannCar(i + 1))

        # What to do if shut down
        rospy.on_shutdown(self.shutdown)
        rospy.spin()

    def shutdown(self):
        rospy.loginfo("Stop Car")
        for i in range(self.numberOfCars):
            self.cars[i].ackermann.unregister()
            self.cars[i].pub_vel_left_rear_wheel.publish(0)
            self.cars[i].pub_vel_right_rear_wheel.publish(0)
            self.cars[i].pub_vel_left_front_wheel.publish(0)
            self.cars[i].pub_vel_right_front_wheel.publish(0)
            self.cars[i].pub_pos_right_steering_hinge.publish(0)
            self.cars[i].pub_pos_left_steering_hinge.publish(0)
        # sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
        rospy.sleep(1)


if __name__ == '__main__':
    try:
        num = int(sys.argv[1])
        AckermannCar(num)

    except rospy.ROSInterruptException:
        rospy.loginfo("User pressed  Ctrl-C, quit!")


