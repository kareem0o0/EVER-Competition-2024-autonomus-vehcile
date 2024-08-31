#!/usr/bin/env python

import rospy 
import numpy as np
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, String, Float32MultiArray
from tf.transformations import euler_from_quaternion

# Tunable parameters
look_ahead = 2  # Look-ahead distance
wheel_base = 2.26963  # Vehicle's wheelbase
human_stop_distance = 14  # Distance to stop for a human
lane_change_distance = 10  # Distance to change lane for a car or cone
lane_change_cooldown = 8  # Cooldown period for lane changes (seconds)
speed = 0.15  # Constant speed

# Global variables
detected_object = ""
x = y = 0
flag = "R"
last_lane_change_time = 0

def init_node():
    global cmd_pub, steering_pub, brake_pub, last_lane_change_time

    rospy.init_node("lane_change_control", anonymous=True)

    rospy.Subscriber('/odom', Odometry, call_back_odom)
    rospy.Subscriber('/detected_labels', String, manage_object)
    rospy.Subscriber('/lidar_points', Float32MultiArray, manage_lidar)
    cmd_pub = rospy.Publisher('/cmd_vel', Float64, queue_size=10)
    steering_pub = rospy.Publisher('/SteeringAngle', Float64, queue_size=10)
    brake_pub = rospy.Publisher('/brakes', Float64, queue_size=10)

    last_lane_change_time = rospy.get_time()
    
    rate = rospy.Rate(10)
    rate.sleep()

def emergency_stop():
    cmd_pub.publish(0)
    steering_pub.publish(0)
    brake_pub.publish(1)
    rospy.sleep(3)

def manage_object(msg):
    global detected_object
    detected_object = msg.data

def manage_lidar(msg):
    global x, y
    x = msg.data[0]
    y = msg.data[1]

def flag_control():
    global detected_object, x, y, flag, last_lane_change_time

    if detected_object == "person" and y < human_stop_distance and abs(x) < 3:  # TUNABLE
        emergency_stop()
    elif detected_object in ["car", "cone"]:
        current_time = rospy.get_time()
        if np.sqrt(x**2 + y**2) <= lane_change_distance and current_time - last_lane_change_time > lane_change_cooldown:  # TUNABLE
            change_lane()
            last_lane_change_time = current_time
        elif np.sqrt(x**2 + y**2) <= lane_change_distance:
            emergency_stop

def change_lane():
    global flag
    if flag == "R":
        flag = "L"
    else:
        flag = "R"

def call_back_odom(odom):
    global flag
    waypoints = [(0.0, 0.0)]
    C_pose = [0.0, 0.0]

    orientation_q = odom.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (_, _, yaw) = euler_from_quaternion(orientation_list)
    C_pose[0] = odom.pose.pose.position.x
    C_pose[1] = odom.pose.pose.position.y

    flag_control()

    if flag == "L":
        waypoints = L_path
    elif flag == "R":
        waypoints = R_path
        
    for point in waypoints:
        if point[1] > (C_pose[1] + look_ahead) and point[1] <= (C_pose[1] + look_ahead + 6):

            dy = abs(point[1] - C_pose[1])
            dx = C_pose[0] - point[0] 
            local_x = np.cos(yaw) * dy + np.sin(yaw) * dx
            local_y = -np.sin(yaw) * dy + np.cos(yaw) * dx

            curvature = 2 * local_y / (local_x ** 2 + local_y ** 2)
            steering = np.arctan(curvature * wheel_base) * 180 / np.pi
            if abs(steering) <= 1:  # TUNABLE
                steering = 0

            steering_pub.publish(steering)
            cmd_pub.publish(speed)  # TUNABLE
            brake_pub.publish(0)

if __name__ == '__main__':
    try:
        y_values = np.linspace(0, 200, 200)
        x_values = y_values * 0
        R_path = list(zip(x_values, y_values))
        x_values[:] = 4
        L_path = list(zip(x_values, y_values))

        init_node()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
