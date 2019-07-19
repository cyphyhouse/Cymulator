#!/usr/bin/env python3
from nav_msgs.msg       import Odometry
from geometry_msgs.msg  import Point, PointStamped, Twist, Pose, PoseStamped
from std_msgs.msg       import Float64, String
from ackermann_msgs.msg import AckermannDriveStamped
import rospy, sys, time


# TODO: this file needs to be changed corresponding to the new MPC controller
class Car:
    def __init__(self, car_id, goals):

        # Car attributes
        self._x = 0.0
        self._y = 0.0
        self._theta = 0.
        self.id = car_id

        # Goal attributes
        self.complete = 0
        self.goal = Point()
        self.goal.x = goals[0]
        self.goal.y = goals[1]
        self.lastsignal = time.time()

        # Wheels controller attributes (ROS)
        identification = '/car' + str(car_id)
        self.pub_vel_left_rear_wheel = rospy.Publisher(identification + '/racecar/left_rear_wheel_velocity_controller/command', Float64, queue_size=1)
        self.pub_vel_right_rear_wheel = rospy.Publisher(identification + '/racecar/right_rear_wheel_velocity_controller/command', Float64, queue_size=1)
        self.pub_vel_left_front_wheel = rospy.Publisher(identification + '/racecar/left_front_wheel_velocity_controller/command', Float64, queue_size=1)
        self.pub_vel_right_front_wheel = rospy.Publisher(identification + '/racecar/right_front_wheel_velocity_controller/command', Float64, queue_size=1)
        self.pub_pos_left_steering_hinge = rospy.Publisher(identification + '/racecar/left_steering_hinge_position_controller/command', Float64, queue_size=1)
        self.pub_pos_right_steering_hinge = rospy.Publisher(identification + '/racecar/right_steering_hinge_position_controller/command', Float64, queue_size=1)
        
        self.ackermann = rospy.Subscriber(identification + "/ackermann_cmd", AckermannDriveStamped, self.set_throttle)
        self.sub_pos = rospy.Subscriber(identification + "/ground_truth/state", Odometry, self.newPos)
        # self.sub_goal = rospy.Subscriber(identification + "/waypoint", PoseStamped, self.newGoal)
        
        # Outter Interface 
        self.pub_reach = rospy.Publisher(identification + '/reached', String, queue_size=1)
        self.pub_position = rospy.Publisher(identification + '/vrpn_client_node', PoseStamped, queue_size=1)


        # Behavior defining
        rospy.on_shutdown(self.shutdown)
        # self.controller()


    def set_throttle(self, data):
        throttle = data.drive.speed/0.1
        steer = data.drive.steering_angle
        rospy.loginfo("Set throttle: %f, set steer: %f. ", throttle, steer)
        self.pub_pos_right_steering_hinge.publish(steer)
        self.pub_pos_left_steering_hinge.publish(steer)
        self.pub_vel_left_rear_wheel.publish(throttle)
        self.pub_vel_right_rear_wheel.publish(throttle)
        self.pub_vel_left_front_wheel.publish(throttle)
        self.pub_vel_right_front_wheel.publish(throttle)
        self.lastsignal = time.time()
      


    def newPos(self, msg):
        '''
            Callback function to get car's current location
            :param msg: Odometry type msg that contains position and orientation of a car
            :return: Nothing
        '''
        self._x = msg.pose.pose.position.x
        self._y = msg.pose.pose.position.y

        # TODO: Translating the location
        pose = PoseStamped()
        pose.pose = msg.pose.pose
        # print(pose)
        self.pub_position.publish(pose)

    def controller(self):
        r = rospy.Rate(30)
        
        while True:
            r.sleep()


            

    def stop(self):
        self.pub_vel_left_rear_wheel.publish(0)
        self.pub_vel_right_rear_wheel.publish(0)
        self.pub_vel_left_front_wheel.publish(0)
        self.pub_vel_right_front_wheel.publish(0)
        self.pub_pos_right_steering_hinge.publish(0)
        self.pub_pos_left_steering_hinge.publish(0)

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



if __name__ == '__main__':
    rospy.init_node('car', anonymous=True)
    Car(1, [0,0])
    Car.controller(Car)



