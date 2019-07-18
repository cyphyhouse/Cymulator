#!/usr/bin/env python3

import rospy, sys, os, signal, multiprocessing, threading
from nav_msgs.msg       import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg  import Point, Twist
from geometry_msgs.msg  import PoseStamped, Pose
from std_msgs.msg       import Float64, String
from math               import atan2, sqrt



class Car():
    def __init__(self, car_id, goals):
        '''
        Constructor of Drone object
        :param number: the number/index of this drone
        '''
        # Robot's position and orientation inforamtion
  
        self.shutdown_flag = 0
       
        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0

        self.id = car_id
        self.complete = 0

        self.goal = Point()
        self.goal.x = goals[0]
        self.goal.y = goals[1]

        # Set up subscriber and publisher
        my_number = "/car" + str(car_id)
        
        self.pub_vel_left_rear_wheel = rospy.Publisher(my_number + '/racecar/left_rear_wheel_velocity_controller/command', Float64, queue_size=1)
        self.pub_vel_right_rear_wheel = rospy.Publisher(my_number + '/racecar/right_rear_wheel_velocity_controller/command', Float64, queue_size=1)
        self.pub_vel_left_front_wheel = rospy.Publisher(my_number + '/racecar/left_front_wheel_velocity_controller/command', Float64, queue_size=1)
        self.pub_vel_right_front_wheel = rospy.Publisher(my_number + '/racecar/right_front_wheel_velocity_controller/command', Float64, queue_size=1)

        self.pub_pos_left_steering_hinge = rospy.Publisher(my_number + '/racecar/left_steering_hinge_position_controller/command', Float64, queue_size=1)
        self.pub_pos_right_steering_hinge = rospy.Publisher(my_number + '/racecar/right_steering_hinge_position_controller/command', Float64, queue_size=1)

        self.pub_reach = rospy.Publisher(my_number + '/reached', String, queue_size=1)
        self.sub_pos = rospy.Subscriber(my_number + "/ground_truth/state", Odometry, self.newPos)
        self.sub_goal = rospy.Subscriber(my_number + "/waypoint", PoseStamped, self.newGoal)

        rospy.on_shutdown(self.shutdown)


    def newGoal(self, msg):
        new_goal = Point()
        # new_goal.x = msg.data[0]
        # new_goal.y = msg.data[1]
        new_goal.x = msg.pose.position.x
        new_goal.y = msg.pose.position.y
        print("new goal recieved: ",new_goal)

        self.goal = new_goal
        self.complete = 0


    def newPos(self, msg):
        '''
            Callback function to get car's current location
            :param msg: Odometry type msg that contains position and orientation of a car
            :return: Nothing
        '''
        self._x = msg.pose.pose.position.x
        self._y = msg.pose.pose.position.y

        quat = msg.pose.pose.orientation
        (_, _, self._theta) = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])


    def goto(self):
        '''
            The actual goto method that drives the drones towards goal points
            :param goals: the list of goal points
            :return: 1 -- if succeed
        '''
        rospy.loginfo("Ready to move cars. To stop Cars , press CTRL + C")
        r = rospy.Rate(30)


        rospy.loginfo("Car%d is going to (%f, %f)", self.id, self.goal.x, self.goal.y)

        while not rospy.is_shutdown() and not self.shutdown_flag:
            # TODO: car's goto method also needs support of queued waypoints; may follow that same technique of drones
            if self.complete:
                r.sleep()
                continue

            diff_x = self.goal.x - self._x
            diff_y = self.goal.y - self._y
            angle_to_goal = atan2(diff_y, diff_x)
            self.pub_reach.publish("False")
            print(sqrt(diff_x*diff_x + diff_y*diff_y) )

            # rospy.loginfo("Angle to goal %f", angle_to_goal)
            # rospy.loginfo("Car%d is currently at (%f, %f)", i+1, self.cars[i]._x, self.cars[i]._y)

            if sqrt(diff_x*diff_x + diff_y*diff_y) < 0.2:
                left_rear_wheel = 0
                right_rear_wheel = 0
                left_front_wheel = 0
                right_front_wheel = 0
                right_steering = 0
                left_steering = 0
                self.complete = 1
                print("--------------------reached-------------------")
                self.pub_reach.publish("True")

            elif angle_to_goal - self._theta > 0.1:
                # rospy.loginfo("Left turn at (%f, %f)", self.cars[i]._x, self.cars[i]._y)
                left_steering = 0.3
                right_steering = 0.3
                left_rear_wheel = 5
                right_rear_wheel = 5
                left_front_wheel = 5
                right_front_wheel = 5
            elif angle_to_goal - self._theta < -0.1:
                # rospy.loginfo("Right turn at (%f, %f)", self.cars[i]._x, self.cars[i]._y)
                left_steering = -0.3
                right_steering = -0.3
                left_rear_wheel = 5
                right_rear_wheel = 5
                left_front_wheel = 5
                right_front_wheel = 5
            else:
                # rospy.loginfo("Go straight at (%f, %f)", self.cars[i]._x, self.cars[i]._y)
                left_rear_wheel = 10
                right_rear_wheel = 10
                left_front_wheel = 10
                right_front_wheel = 10
                right_steering = 0
                left_steering = 0

            self.pub_vel_left_rear_wheel.publish(left_rear_wheel)
            self.pub_vel_right_rear_wheel.publish(right_rear_wheel)
            self.pub_vel_left_front_wheel.publish(left_front_wheel)
            self.pub_vel_right_front_wheel.publish(right_front_wheel)
            self.pub_pos_right_steering_hinge.publish(right_steering)
            self.pub_pos_left_steering_hinge.publish(left_steering)
            r.sleep()
        
        self.shutdown()

    def shutdown(self):
        '''
            Stop all cars when rospy shuts down
            :return: Nothing
        '''
        rospy.loginfo("Stop %d Car", self.id)
        self.pub_vel_left_rear_wheel.publish(0)
        self.pub_vel_right_rear_wheel.publish(0)
        self.pub_vel_left_front_wheel.publish(0)
        self.pub_vel_right_front_wheel.publish(0)
        self.pub_pos_right_steering_hinge.publish(0)
        self.pub_pos_left_steering_hinge.publish(0)
        # sleep just makes sure cars receives the stop command prior to shutting down the script
        rospy.loginfo("complete stop car")
        rospy.sleep(1)




