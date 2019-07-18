#!/usr/bin/env python3

'''
    Use ros messages to drive drones
    Involves physics
    NOTE: To publish new waypoint, use this format: rostopic pub /drone1/goals std_msgs/Float32MultiArray "data: [0.0, 1.0, 2.0]"
'''

import rospy, sys
from geometry_msgs.msg      import PoseStamped, Pose, Point, Twist
from math                   import sqrt
from hector_uav_msgs.srv    import EnableMotors
from nav_msgs.msg           import Odometry
from tf.transformations     import euler_from_quaternion


class Drone:
    def __init__(self, drone_id, goal, wpQueued=False):
        '''
        Constructor of Drone object
        :param number: the number/index of this drone
        :param wpQueued: Flag that indicate whether queued waypoints is used
        '''
        # Drone attributes
        self.id = drone_id
        self._x = 0.0
        self._y = 0.0
        self._z = 0.0
        self._theta = 0.0


        # Drone attributes
        self.goals = []
        self.goal = Point()
        self.goal.x = goal[0]
        self.goal.y = goal[1]
        self.goal.z = goal[2]

        self.goals.append(self.goal)

        # Set up subscriber and publisher
        identification = "/drone" + str(drone_id)
        self.sub = rospy.Subscriber(identification + "/ground_truth/state", Odometry, self.newPos)
        self.pub = rospy.Publisher(identification + "/cmd_vel", Twist, queue_size=10)

        if wpQueued:
            self.subGoal = rospy.Subscriber(identification + "/goals", PoseStamped, self.newGoal)

        # Enable motors using ROS service
        rospy.wait_for_service(identification + '/enable_motors')
        try:
            # Set 'enable_motors' to be True
            enable = rospy.ServiceProxy(identification + '/enable_motors', EnableMotors)
            resp = enable(True)
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed with %s", e)


        rospy.on_shutdown(self.shutdown)


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
        new_goal.x = msg.pose.position.x
        new_goal.y = msg.pose.position.y
        new_goal.z = msg.pose.position.z
        self.goals.append(new_goal)


    def controller(self):
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

            curGoal = self.goals[0]
            diff_x = curGoal.x - self._x
            diff_y = curGoal.y - self._y
            diff_z = curGoal.z - self._z
            # rospy.loginfo("Drone %d has distance away from waypoint is (%f, %f, %f)", self.id, diff_x, diff_y, diff_z) # sqrt(diff_x*diff_x + diff_y*diff_y + diff_z*diff_z))
            
            # Waypoint check
            if abs(diff_x) < 0.25 and abs(diff_y) < 0.25 and abs(diff_z) < 0.25:
                rospy.loginfo("Drone%d reaches a waypoint", self.id)
                if len(self.goals) > 1:
                    self.goals.pop(0)
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
            self.pub.publish(move_cmd)

            r.sleep()

    def shutdown(self):
        '''
        Stop all drones when rospy shuts down
        :return: Nothing
        '''
        rospy.loginfo("Stop Drones")
        # a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop Drones
        self.pub.publish(Twist())
        # sleep just makes sure Drones receives the stop command prior to shutting down the script

        rospy.sleep(1)


if __name__ == '__main__': 
    try:
        rospy.init_node('Drone_Test', anonymous=True)

        id = 1
        goals = [3,4,5]
        Drone(id, goals)
        rospy.sleep(1)

    except rospy.ROSInterruptException:
        rospy.loginfo("User pressed  Ctrl-C, quit!")