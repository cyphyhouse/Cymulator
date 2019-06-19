#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PointStamped, Twist, Pose
from std_msgs.msg import Float64
from math import atan2, sqrt
from ackermann_msgs.msg import AckermannDriveStamped

class Car():
    def __init__(self):
        rospy.init_node("Ackermann_control", anonymous=True)
        self.goal_sent = False

        # What to do if shut down
        rospy.on_shutdown(self.shutdown)

        # Set up subscriber and publisher
        self.pub_vel_left_rear_wheel = rospy.Publisher('/racecar/left_rear_wheel_velocity_controller/command', Float64, queue_size=1)
        self.pub_vel_right_rear_wheel = rospy.Publisher('/racecar/right_rear_wheel_velocity_controller/command', Float64, queue_size=1)
        self.pub_vel_left_front_wheel = rospy.Publisher('/racecar/left_front_wheel_velocity_controller/command', Float64, queue_size=1)
        self.pub_vel_right_front_wheel = rospy.Publisher('/racecar/right_front_wheel_velocity_controller/command', Float64, queue_size=1)

        self.pub_pos_left_steering_hinge = rospy.Publisher('/racecar/left_steering_hinge_position_controller/command', Float64, queue_size=1)
        self.pub_pos_right_steering_hinge = rospy.Publisher('/racecar/right_steering_hinge_position_controller/command', Float64, queue_size=1)



        rospy.loginfo("Subscribe ackermann message!")
        self.ackermann = rospy.Subscriber("/ackermann_cmd", AckermannDriveStamped, self.set_throttle)

        rospy.spin()

    def set_throttle(self, data):
        # if not self.goal_sent:
        self.goal_sent = True
        rospy.loginfo("Send goal!")
        pub_goal = rospy.Publisher("/car_goal/goal", PointStamped, queue_size=1)
        goal = PointStamped()
        goal.point.x = 10
        goal.point.y = 20
        goal.header.frame_id = "1"
        pub_goal.publish(goal)

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

    def shutdown(self):
        rospy.loginfo("Stop Car")
        self.pub_vel_left_rear_wheel.publish(0)
        self.pub_vel_right_rear_wheel.publish(0)
        self.pub_vel_left_front_wheel.publish(0)
        self.pub_vel_right_front_wheel.publish(0)
        self.pub_pos_right_steering_hinge.publish(0)
        self.pub_pos_left_steering_hinge.publish(0)
        # sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        Car()

    except rospy.ROSInterruptException:
        rospy.loginfo("User pressed  Ctrl-C, quit!")


