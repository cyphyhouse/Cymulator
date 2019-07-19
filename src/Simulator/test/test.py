import rospy, time
from std_msgs.msg import Float64
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg  import PoseStamped

rospy.init_node('controller', anonymous=True)

# my_number = '/car1'
# pub_vel_left_rear_wheel = rospy.Publisher(my_number + '/racecar/left_rear_wheel_velocity_controller/command', Float64, queue_size=1)
# pub_vel_right_rear_wheel = rospy.Publisher(my_number + '/racecar/right_rear_wheel_velocity_controller/command', Float64, queue_size=1)
# pub_vel_left_front_wheel = rospy.Publisher(my_number + '/racecar/left_front_wheel_velocity_controller/command', Float64, queue_size=1)
# pub_vel_right_front_wheel = rospy.Publisher(my_number + '/racecar/right_front_wheel_velocity_controller/command', Float64, queue_size=1)
# pub_pos_left_steering_hinge = rospy.Publisher(my_number + '/racecar/left_steering_hinge_position_controller/command', Float64, queue_size=1)
# pub_pos_right_steering_hinge = rospy.Publisher(my_number + '/racecar/right_steering_hinge_position_controller/command', Float64, queue_size=1)

# for i in range(0, 3):
#     pub_vel_left_rear_wheel.publish(0)
#     pub_vel_right_rear_wheel.publish(0)
#     pub_vel_left_front_wheel.publish(0)
#     pub_vel_right_front_wheel.publish(0)
#     pub_pos_right_steering_hinge.publish(0)
#     pub_pos_left_steering_hinge.publish(0)
#     time.sleep(0.5)

# for i in range(0, 20):

#     pub_vel_left_rear_wheel.publish(20)
#     pub_vel_right_rear_wheel.publish(20)
#     pub_vel_left_front_wheel.publish(20)
#     pub_vel_right_front_wheel.publish(20)
#     pub_pos_right_steering_hinge.publish(0)
#     pub_pos_left_steering_hinge.publish(0)
#     time.sleep(0.5)

# for i in range(0, 20):

#     pub_vel_left_rear_wheel.publish(-10)
#     pub_vel_right_rear_wheel.publish(-10)
#     pub_vel_left_front_wheel.publish(-10)
#     pub_vel_right_front_wheel.publish(-10)
#     pub_pos_right_steering_hinge.publish(0.3)
#     pub_pos_left_steering_hinge.publish(0.3)
#     time.sleep(0.5)


# pub_vel_left_rear_wheel.publish(0)
# pub_vel_right_rear_wheel.publish(0)
# pub_vel_left_front_wheel.publish(0)
# pub_vel_right_front_wheel.publish(0)
# pub_pos_right_steering_hinge.publish(0)
# pub_pos_left_steering_hinge.publish(0)


# ----------------------------------------------------------
# drive_pub = rospy.Publisher( '/car1/ackermann_cmd', AckermannDriveStamped, queue_size=1)

# for i in range(10):
#     drive_msg = AckermannDriveStamped()
#     drive_msg.drive.speed = 1
#     drive_msg.drive.steering_angle = 0

#     drive_pub.publish(drive_msg)
#     time.sleep(0.5)


drive_pub = rospy.Publisher( '/car2/waypoint', PoseStamped, queue_size=1)

pose = PoseStamped()
pose.header.stamp = rospy.Time.now()
pose.header.frame_id = "0"
pose.pose.position.x = 7.0
pose.pose.position.y = 5.0
pose.pose.position.z = 0.0
            
pose.pose.orientation.x = 0.0
pose.pose.orientation.y = 0.0
pose.pose.orientation.z = 0.0
pose.pose.orientation.w = 0.0
        
drive_pub.publish(pose)
time.sleep(1)


pose = PoseStamped()
pose.header.stamp = rospy.Time.now()
pose.header.frame_id = "0"
pose.pose.position.x = 45.0
pose.pose.position.y = -5.0
pose.pose.position.z = 0.0
            
pose.pose.orientation.x = 0.0
pose.pose.orientation.y = 0.0
pose.pose.orientation.z = 0.0
pose.pose.orientation.w = 0.0
        
drive_pub.publish(pose)
time.sleep(1)

pose = PoseStamped()
pose.header.stamp = rospy.Time.now()
pose.header.frame_id = "0"
pose.pose.position.x = -3.0
pose.pose.position.y = -5.0
pose.pose.position.z = 0.0
            
pose.pose.orientation.x = 0.0
pose.pose.orientation.y = 0.0
pose.pose.orientation.z = 0.0
pose.pose.orientation.w = 0.0
        
drive_pub.publish(pose)
time.sleep(1)




pose = PoseStamped()
pose.header.stamp = rospy.Time.now()
pose.header.frame_id = "1"
pose.pose.position.x = -5.0
pose.pose.position.y = 5.0
pose.pose.position.z = 0.0
        
pose.pose.orientation.x = 0.0
pose.pose.orientation.y = 0.0
pose.pose.orientation.z = 0.0
pose.pose.orientation.w = 0.0
drive_pub.publish(pose)


# rostopic pub /waypoint/waypoint geometry_msgs/PoseStamped '{header: {stamp: {}, frame_id: "1"}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}'