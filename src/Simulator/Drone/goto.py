#!/usr/bin/env python3

'''
    Use ros messages to drive drones
    Involves physics
'''

import rospy
from geometry_msgs.msg import Point, Twist
from math import sqrt
import sys
from hector_uav_msgs.srv import EnableMotors
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


class Drone:
    def __init__(self, number):
        # Drone's position and orientation inforamtion
        self._x = 0.0
        self._y = 0.0
        self._z = 0.0
        self._theta = 0.0
        self.goal = Point()

        # Set up subscriber and publisher
        my_number = "/drone" + str(number)
        self.sub = rospy.Subscriber(my_number + "/ground_truth/state", Odometry, self.newPos)
        self.pub = rospy.Publisher(my_number + "/cmd_vel", Twist, queue_size=10)

        # Enable motors using ROS service
        rospy.wait_for_service(my_number + '/enable_motors')
        try:
            # Set 'enable_motors' to be True
            enable = rospy.ServiceProxy(my_number + '/enable_motors', EnableMotors)
            resp = enable(True)
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed with %s", e)

    def newPos(self, msg):
        self._x = msg.pose.pose.position.x
        self._y = msg.pose.pose.position.y
        self._z = msg.pose.pose.position.z

        quat = msg.pose.pose.orientation
        (_, _, self._theta) = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])


class GoTo:
    '''
    This is the class that handles GOTO functionality of drones
    '''
    def __init__(self, num, goals):
        '''
        Constructor
        :param num: number of drones
        :param goals: the goals that drones are flying to
        '''
        self.numberOfDrones = num
        self.drones = []
        self.complete = []
        self.success = 0
        # Setup drones
        for i in range(self.numberOfDrones):
            self.drones.append(Drone(i+1))
            self.complete.append(0)

        # What to do if shut down
        rospy.on_shutdown(self.shutdown)
        self.success = self.goto(goals)

    def goto(self, goals):
        rospy.loginfo("Ready to move. To stop Drone , press CTRL + C")
        r = rospy.Rate(10)  # Setup ROS spin rate
        move_cmd = Twist()  # Twist messages

        # Set up goal
        for i in range(self.numberOfDrones):
            self.drones[i].goal.x = goals[i][0]
            self.drones[i].goal.y = goals[i][1]
            self.drones[i].goal.z = goals[i][2]
            rospy.loginfo("Drone %d is going to (%f, %f, %f)", i+1, self.drones[i].goal.x, self.drones[i].goal.y, self.drones[i].goal.z)

        while not rospy.is_shutdown():
            if sum(self.complete) == self.numberOfDrones:
                return 1

            # Simple controller code for drones
            # TODO: Controller might need to be changed
            for i in range(self.numberOfDrones):
                # rospy.loginfo("Drone %d is at (%f, %f, %f)", i+1, self.drones[i]._x, self.drones[i]._y, self.drones[i]._z)
                diff_x = self.drones[i].goal.x - self.drones[i]._x
                diff_y = self.drones[i].goal.y - self.drones[i]._y
                diff_z = self.drones[i].goal.z - self.drones[i]._z

                if sqrt(diff_x*diff_x + diff_y*diff_y + diff_z*diff_z) < 0.05:
                    self.complete[i] = 1 
                else:
                    if abs(diff_x) > 0.1:
                        if diff_x > 0:
                            move_cmd.linear.x = 0.5
                        else:
                            move_cmd.linear.x = -0.5
                    else:
                        move_cmd.linear.x = 0.0

                    if abs(diff_y) > 0.1:
                        if diff_y > 0:
                            move_cmd.linear.y = 0.5
                        else:
                            move_cmd.linear.y = -0.5
                    else:
                        move_cmd.linear.y = 0.0

                    if abs(diff_z) > 0.1:
                        if diff_z > 0:
                            move_cmd.linear.z = 0.5
                        else:
                            move_cmd.linear.z = 0.0
                    else:
                        move_cmd.linear.z = 0.0
            
                self.drones[i].pub.publish(move_cmd)

            r.sleep()

    def shutdown(self):
        '''
        Stop all drones when rospy shuts down
        :return: Nothing
        '''
        rospy.loginfo("Stop Drones")
        # a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop Drones

        for i in range(self.numberOfDrones):
            self.drones[i].pub.publish(Twist())
        # sleep just makes sure Drones receives the stop command prior to shutting down the script
            
        rospy.sleep(1)


if __name__ == '__main__': 
    try:
        rospy.init_node('Drone_Test', anonymous=True)
        sys.path.append('..')
        from util import parse_goal_pose
        num = int(sys.argv[1])
        goals = parse_goal_pose(num, sys.argv[2:], 'drone')

        navigator = GoTo(num, goals)

        if navigator.success:
            rospy.loginfo("Yep, we made it!")
        else:
            rospy.loginfo("Something is wrong")
            
        rospy.sleep(1)
    except rospy.ROSInterruptException:
        rospy.loginfo("User pressed  Ctrl-C, quit!")