import rospy
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from hector_uav_msgs.srv import EnableMotors
from math import sqrt

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


class Listener:
    def __init__(self, num):
        rospy.init_node("Listener_Drone", anonymous=True)
        self.numberOfDrones = num
        self.drones = []
        self.complete = []
        self.success = 0
        self.goal_received = False

        # Setup drones
        # for i in range(self.numberOfDrones):
        #     self.drones.append(Drone(i + 1))
        #     self.complete.append(0)
        self.drone = Drone(1)
        self.listen = rospy.Subscriber("/drone1/drive_cmd", Twist, self.set_throttle)
        # What to do if shut down
        rospy.on_shutdown(self.shutdown)
        rospy.spin()

    def set_throttle(self, move_cmd):
        # if not self.goal_sent:
        self.goal_received = True
        # pub_goal = rospy.Publisher("/car_goal/goal", PointStamped, queue_size=1)
        # goal = PointStamped()
        rospy.loginfo("Command received!")

        self.drone.pub.publish(move_cmd)

    def shutdown(self):
        rospy.loginfo("Stop Car")
        self.drone.pub.publish(0)
        # sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
        rospy.sleep(1)


if __name__ == '__main__':
    try:
        Listener(1)

    except rospy.ROSInterruptException:
        rospy.loginfo("User pressed  Ctrl-C, quit!")