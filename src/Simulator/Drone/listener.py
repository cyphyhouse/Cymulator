import rospy
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import sqrt


class Listener:
    def __init__(self, num):
        rospy.init_node("Listener Drone", anonymous=True)
        # Drone's position and orientation information
        self._x = 0.0
        self._y = 0.0
        self._z = 0.0
        self._theta = 0.0
        self.goal = Point()
        self.goal_received = False

        # What to do if shut down
        rospy.on_shutdown(self.shutdown)

        # Set up subscriber and publisher
        my_number = "/drone" + str(num)
        self.pos = rospy.Subscriber(my_number + "/ground_truth/state", Odometry, self.newPos)

        rospy.loginfo("Subscribe to move!")
        self.listen = rospy.Subscriber(my_number + "/receive_cmd", Twist, self.set_throttle)

        self.pub = rospy.Publisher(my_number + "/cmd_vel", Twist, queue_size=10)

        rospy.spin()

    def set_throttle(self, move_cmd):
        # if not self.goal_sent:
        self.goal_received = True
        rospy.loginfo("Send goal!")
        # pub_goal = rospy.Publisher("/car_goal/goal", PointStamped, queue_size=1)
        # goal = PointStamped()
        rospy.loginfo("Command received!")

        self.pub.publish(move_cmd)

    def newPos(self, msg):
        self._x = msg.pose.pose.position.x
        self._y = msg.pose.pose.position.y
        self._z = msg.pose.pose.position.z

        quat = msg.pose.pose.orientation
        (_, _, self._theta) = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])

    def shutdown(self):
        rospy.loginfo("Stop Car")
        self.pub.publish(0)
        # sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
        rospy.sleep(1)


if __name__ == '__main__':
    try:
        Listener(1)

    except rospy.ROSInterruptException:
        rospy.loginfo("User pressed  Ctrl-C, quit!")