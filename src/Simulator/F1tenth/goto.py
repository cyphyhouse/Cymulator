#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2, sqrt
from car import Car
import sys

class GoTo():
    def __init__(self, num, goals):

        self.numberOfCars = num
        self.cars = []
        self.complete = []
        self.success = 0
        # Setup Cars
        for i in range(self.numberOfCars):
            self.cars.append(Car(i+1))
            self.complete.append(0)

        # What to do if shut down
        rospy.on_shutdown(self.shutdown)
        self.success = self.goto(goals)

    def goto(self, goals):
        rospy.loginfo("Ready to move cars. To stop Cars , press CTRL + C")
        r = rospy.Rate(10)

        # Set up goal
        for i in range(self.numberOfCars):
            self.cars[i].goal.x = goals[i][0]
            self.cars[i].goal.y = goals[i][1]
            rospy.loginfo("Car%d is going to (%f, %f)", i+1, self.cars[i]._x, self.cars[i]._y)

        while not rospy.is_shutdown():
            if sum(self.complete) == self.numberOfCars:
                return 1
            for i in range(self.numberOfCars):
                diff_x = self.cars[i].goal.x - self.cars[i]._x
                diff_y = self.cars[i].goal.y - self.cars[i]._y
                angle_to_goal = atan2(diff_y, diff_x)

                # rospy.loginfo("Angle to goal %f", angle_to_goal)
                rospy.loginfo("Car%d is currently at (%f, %f)", i+1, self.cars[i]._x, self.cars[i]._y)

                if sqrt(diff_x*diff_x + diff_y*diff_y) < 0.2:
                    self.cars[i].pub_vel_left_rear_wheel.publish(0)
                    self.cars[i].pub_vel_right_rear_wheel.publish(0)
                    self.cars[i].pub_vel_left_front_wheel.publish(0)
                    self.cars[i].pub_vel_right_front_wheel.publish(0)
                    self.cars[i].pub_pos_right_steering_hinge.publish(0)
                    self.cars[i].pub_pos_left_steering_hinge.publish(0)
                    self.complete[i] = 1
                elif angle_to_goal - self.cars[i]._theta > 0.1:
                    # rospy.loginfo("Left turn at (%f, %f)", self.cars[i]._x, self.cars[i]._y)
                    self.cars[i].pub_pos_left_steering_hinge.publish(0.3)
                    self.cars[i].pub_pos_right_steering_hinge.publish(0.3)
                    self.cars[i].pub_vel_left_rear_wheel.publish(5)
                    self.cars[i].pub_vel_right_rear_wheel.publish(5)
                    self.cars[i].pub_vel_left_front_wheel.publish(5)
                    self.cars[i].pub_vel_right_front_wheel.publish(5)
                elif angle_to_goal - self.cars[i]._theta < -0.1:
                    # rospy.loginfo("Right turn at (%f, %f)", self.cars[i]._x, self.cars[i]._y)
                    self.cars[i].pub_pos_right_steering_hinge.publish(-0.3)
                    self.cars[i].pub_pos_left_steering_hinge.publish(-0.3)
                    self.cars[i].pub_vel_left_rear_wheel.publish(5)
                    self.cars[i].pub_vel_right_rear_wheel.publish(5)
                    self.cars[i].pub_vel_left_front_wheel.publish(5)
                    self.cars[i].pub_vel_right_front_wheel.publish(5)
                else:
                    # rospy.loginfo("Go straight at (%f, %f)", self.cars[i]._x, self.cars[i]._y)
                    self.cars[i].pub_vel_left_rear_wheel.publish(10)
                    self.cars[i].pub_vel_right_rear_wheel.publish(10)
                    self.cars[i].pub_vel_left_front_wheel.publish(10)
                    self.cars[i].pub_vel_right_front_wheel.publish(10)
                    self.cars[i].pub_pos_right_steering_hinge.publish(0)
                    self.cars[i].pub_pos_left_steering_hinge.publish(0)

            r.sleep()

    def shutdown(self):
        rospy.loginfo("Stop Car")
        for i in range(self.numberOfCars):
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
        rospy.init_node('F1tenth', anonymous=True)
        sys.path.append('..')
        from util import parse_goal_pose
        num = int(sys.argv[1])
        goals = parse_goal_pose(num, sys.argv[2:], 'car')

        navigator = GoTo(num, goals)

        if navigator.success:
            rospy.loginfo("Yep, we made it!")
        else:
            rospy.loginfo("Something is wrong")
            
        rospy.sleep(1)
    except rospy.ROSInterruptException:
        rospy.loginfo("User pressed  Ctrl-C, quit!")