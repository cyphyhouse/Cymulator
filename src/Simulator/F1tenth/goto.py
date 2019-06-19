#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2, sqrt
from car import Car

class GoTo():
    def __init__(self):
        rospy.init_node('F1tenth', anonymous=False)
        self.car = Car()

        # What to do if shut down
        rospy.on_shutdown(self.shutdown)

    def goto(self, goal):
        rospy.loginfo("Ready to move. To stop Robot , press CTRL + C")
        rospy.loginfo("Robot1 goes to (%f, %f)", pos1['x'], pos1['y'])
        r = rospy.Rate(10)

        # Set up goal
        self.car.goal.x = goal['x']
        self.car.goal.y = goal['y']

        while not rospy.is_shutdown():
            diff_x = self.car.goal.x - self.car._x
            diff_y = self.car.goal.y - self.car._y
            angle_to_goal = atan2(diff_y, diff_x)

            rospy.loginfo("Angle to goal %f", angle_to_goal)
            rospy.loginfo("Orientation %f", self.car._theta)

            if sqrt(diff_x*diff_x + diff_y*diff_y) < 0.2:
                return 1
            elif angle_to_goal - self.car._theta > 0.1:
                rospy.loginfo("Left turn at (%f, %f)", self.car._x, self.car._y)
                self.car.pub_pos_left_steering_hinge.publish(0.3)
                self.car.pub_pos_right_steering_hinge.publish(0.3)
                self.car.pub_vel_left_rear_wheel.publish(5)
                self.car.pub_vel_right_rear_wheel.publish(5)
                self.car.pub_vel_left_front_wheel.publish(5)
                self.car.pub_vel_right_front_wheel.publish(5)
            elif angle_to_goal - self.car._theta < -0.1:
                rospy.loginfo("Right turn at (%f, %f)", self.car._x, self.car._y)
                self.car.pub_pos_right_steering_hinge.publish(-0.3)
                self.car.pub_pos_left_steering_hinge.publish(-0.3)
                self.car.pub_vel_left_rear_wheel.publish(5)
                self.car.pub_vel_right_rear_wheel.publish(5)
                self.car.pub_vel_left_front_wheel.publish(5)
                self.car.pub_vel_right_front_wheel.publish(5)
            else:
                rospy.loginfo("Go straight at (%f, %f)", self.car._x, self.car._y)
                self.car.pub_vel_left_rear_wheel.publish(10)
                self.car.pub_vel_right_rear_wheel.publish(10)
                self.car.pub_vel_left_front_wheel.publish(10)
                self.car.pub_vel_right_front_wheel.publish(10)
                self.car.pub_pos_right_steering_hinge.publish(0)
                self.car.pub_pos_left_steering_hinge.publish(0)

            r.sleep()

    def shutdown(self):
        rospy.loginfo("Stop Car")
        self.car.pub_vel_left_rear_wheel.publish(0)
        self.car.pub_vel_right_rear_wheel.publish(0)
        self.car.pub_vel_left_front_wheel.publish(0)
        self.car.pub_vel_right_front_wheel.publish(0)
        self.car.pub_pos_right_steering_hinge.publish(0)
        self.car.pub_pos_left_steering_hinge.publish(0)
        # sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
        rospy.sleep(1)


if __name__ == '__main__': 
    try:
        navigator = GoTo()

        pos1 = {'x': 10, 'y': -10}
        success = navigator.goto(pos1)

        if success:
            rospy.loginfo("Yep, we made it!")
        else:
            rospy.loginfo("Something is wrong")
            
        rospy.sleep(1)
    except:
        rospy.loginfo("User pressed  Ctrl-C, quit!")