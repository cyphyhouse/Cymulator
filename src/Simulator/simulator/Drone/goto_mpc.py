#!/usr/bin/env python3

'''
    Use ros messages to drive drones
    Involves physics
    NOTE: To publish new waypoint, use this format: rostopic pub /drone1/goals std_msgs/Float32MultiArray "data: [0.0, 1.0, 2.0]"
'''

import rospy, sys, queue
from math                   import sqrt
from geometry_msgs.msg      import PoseStamped, Pose, Point, Twist
from hector_uav_msgs.srv    import EnableMotors
from nav_msgs.msg           import Odometry
from tf.transformations     import euler_from_quaternion
from std_msgs.msg           import String
from ackermann_msgs.msg import AckermannDriveStamped

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

        # Waypoint attributes
        self.goal = Point()
        self.goal.x = goal[0]
        self.goal.y = goal[1]
        self.goal.z = goal[2]

        # Controller interface
        identification = "/drone" + str(drone_id)
        self.sub = rospy.Subscriber(identification + "/ground_truth/state", Odometry, self.newPos)
        self.pub = rospy.Publisher(identification + "/cmd_vel", Twist, queue_size=10)

        # Outter interface 
        self.pub_position = rospy.Publisher(identification + '/vrpn_client_node', PoseStamped, queue_size=1)
        self.ackermann = rospy.Subscriber(identification + "/ackermann_cmd", AckermannDriveStamped, self.set_throttle)

        # Enable motors using ROS service
        rospy.wait_for_service(identification + '/enable_motors')
        try:
            # Set 'enable_motors' to be True
            enable = rospy.ServiceProxy(identification + '/enable_motors', EnableMotors)
            resp = enable(True)
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed with %s", e)


        rospy.on_shutdown(self.shutdown)


    def set_throttle(self, data):
        # TODO: get the ackermann_cmd from /ackermann_cmd topic and publish the correponding command to  /cmd_vel
        pass
   

    def newPos(self, msg):
        '''
        Callback function to get drone's current location
        :param msg: Odometry type msg that contains position and orientation of a drone
        :return: Nothing
        '''
        self._x = msg.pose.pose.position.x
        self._y = msg.pose.pose.position.y
        self._z = msg.pose.pose.position.z

        pose = PoseStamped()
        pose.pose = msg.pose.pose
        self.pub_position.publish(pose)




    def controller(self):
        r = rospy.Rate(30)
        
        while True:
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
        rospy.init_node('Drone', anonymous=True)
        Drone(1, [0,0])
        Drone.controller(Drone)

    except rospy.ROSInterruptException:
        rospy.loginfo("User pressed  Ctrl-C, quit!")