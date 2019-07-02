#!/usr/bin/env python3

'''
    Use ros messages to drive drones
    Involves physics
    NOTE: To publish new waypoint, use this format: rostopic pub /drone1/goals std_msgs/Float32MultiArray "data: [0.0, 1.0, 2.0]"
'''

import rospy
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import Float32MultiArray
from math import sqrt
import sys
from hector_uav_msgs.srv import EnableMotors
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


class Drone:
    def __init__(self, number, wpQueued=False):
        '''
        Constructor of Drone object
        :param number: the number/index of this drone
        :param wpQueued: Flag that indicate whether queued waypoints is used
        '''
        # Drone's position and orientation inforamtion
        self._x = 0.0
        self._y = 0.0
        self._z = 0.0
        self._theta = 0.0
        self.goals = []

        # Set up subscriber and publisher
        my_number = "/drone" + str(number)
        self.sub = rospy.Subscriber(my_number + "/ground_truth/state", Odometry, self.newPos)
        self.pub = rospy.Publisher(my_number + "/cmd_vel", Twist, queue_size=10)

        if wpQueued:
            self.subGoal = rospy.Subscriber(my_number + "/goals", Float32MultiArray, self.newGoal)

        # Enable motors using ROS service
        rospy.wait_for_service(my_number + '/enable_motors')
        try:
            # Set 'enable_motors' to be True
            enable = rospy.ServiceProxy(my_number + '/enable_motors', EnableMotors)
            resp = enable(True)
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed with %s", e)

    def newPos(self, msg):
        '''
        Callback function to get drone's current location
        :param msg: Odometry type msg that contains position and orientation of a drone
        :return: Nothing
        '''
        self._x = msg.pose.pose.position.x
        self._y = msg.pose.pose.position.y
        self._z = msg.pose.pose.position.z

        quat = msg.pose.pose.orientation
        (_, _, self._theta) = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])

    def newGoal(self, msg):
        '''
        Callback function to get drone's next waypoint/goal
        :param msg: Float32MultiArray type msg that contains the new waypoint/goal
        :return: Nothing
        '''
        new_goal = Point()
        new_goal.x = msg.data[0]
        new_goal.y = msg.data[1]
        new_goal.z = msg.data[2]
        self.goals.append(new_goal)


class GoTo:
    '''
    This is the class that handles GOTO functionality of drones
    '''
    def __init__(self, num, goals, wpQueued=False):
        '''
        Constructor
        :param num: number of drones
        :param goals: the goals that drones are flying to
        '''

        rospy.loginfo("Drone's goto method gets called! ")
        self.numberOfDrones = num
        self.drones = []
        self.complete = []  # TODO: make it a dictionary, key=pid, value: Bool = whether the drone reaches a waypoint
        self.success = 0
        # Setup drones
        for i in range(self.numberOfDrones):
            self.drones.append(Drone(i+1, wpQueued))
            self.complete.append(0)
            goal = Point()
            goal.x = goals[i][0]
            goal.y = goals[i][1]
            goal.z = goals[i][2]
            self.drones[i].goals.append(goal)
            rospy.loginfo("Drone %d is going to (%f, %f, %f)", i + 1, goal.x, goal.y, goal.z)

        # What to do if shut down
        rospy.on_shutdown(self.shutdown)
        self.success = self.goto(goals)

    def goto(self, goals):
        '''
        The actual goto method that drives the drones towards goal points
        :param goals: the list of goal points
        :return: Nothing
        '''
        rospy.loginfo("Ready to move. To stop Drone , press CTRL + C")
        r = rospy.Rate(10)  # Setup ROS spin rate
        move_cmd = Twist()  # Twist messages

        while not rospy.is_shutdown():
            # Simple controller code for drones
            # TODO: Controller might need to be changed
            for i in range(self.numberOfDrones):
                curGoal = self.drones[i].goals[0]
                diff_x = curGoal.x - self.drones[i]._x
                diff_y = curGoal.y - self.drones[i]._y
                diff_z = curGoal.z - self.drones[i]._z

                rospy.loginfo("Drone %d has distance away from waypoint is (%f, %f, %f)", i+1, diff_x, diff_y, diff_z) # sqrt(diff_x*diff_x + diff_y*diff_y + diff_z*diff_z))

                # Waypoint check
                if abs(diff_x) < 0.25 and abs(diff_y) < 0.25 and abs(diff_z) < 0.25:
                    rospy.loginfo("Drone%d reaches a waypoint", i+1)
                    if len(self.drones[i].goals) > 1:
                        self.drones[i].goals.pop(0)
                else:
                    if abs(diff_x) > 0.5:
                        if diff_x > 0:
                            move_cmd.linear.x = 0.5
                        else:
                            move_cmd.linear.x = -0.5
                    else:
                        if diff_x > 0:
                            move_cmd.linear.x = 0.05
                        else:
                            move_cmd.linear.x = -0.05

                    if abs(diff_y) > 0.5:
                        if diff_y > 0:
                            move_cmd.linear.y = 0.5
                        else:
                            move_cmd.linear.y = -0.5
                    else:
                        if diff_y > 0:
                            move_cmd.linear.y = 0.05
                        else:
                            move_cmd.linear.y = -0.05

                    if abs(diff_z) > 0.5:
                        if diff_z > 0:
                            move_cmd.linear.z = 0.5
                        else:
                            move_cmd.linear.z = -0.5
                    else:
                        if diff_z > 0:
                            move_cmd.linear.z = 0.05
                        else:
                            move_cmd.linear.z = -0.05
            
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

        GoTo(num, goals, True)
            
        rospy.sleep(1)
    except rospy.ROSInterruptException:
        rospy.loginfo("User pressed  Ctrl-C, quit!")