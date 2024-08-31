#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float64, String, Float32MultiArray
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

# Global parameters
global wheel_base
global lookAhead
global current_position
global maxWheelVelocity
global path_flag
global object_detected
global lane_change_cooldown
global last_lane_change_time

maxWheelVelocity = 114.3202437
wheel_base = 2.26963
lookAhead = 2  # TUNABLE
current_position = [0, 0]
radius_1 = 18  # Radius of the first circular path
radius_2 = 23  # Radius of the second circular path
path_flag = 1  # 1 for path with radius_1, 2 for path with radius_2
object_detected = ""
TTC = float('inf')
x = y = 0

# Thresholds
human_stop_distance = 11 # Distance to stop for human
car_cone_stop_distance = 11  # Distance to stop for car or cone
lane_change_distance = 10  # Distance to change lane for car or cone
lane_change_cooldown = 7  # Cooldown period in seconds

def init_node():
    global cmd_pub, steering_pub, brake_pub, last_lane_change_time

    rospy.init_node("pure_pursuit_control", anonymous=True)
    odom_sub = rospy.Subscriber('/odom', Odometry, calculate_lookAhead_waypoint)
    cmd_pub = rospy.Publisher('/cmd_vel', Float64, queue_size=10)
    steering_pub = rospy.Publisher('/SteeringAngle', Float64, queue_size=10)
    brake_pub = rospy.Publisher('/brakes', Float64, queue_size=10)
    object_sub = rospy.Subscriber('/detected_labels', String, manage_object)
    lidar_sub = rospy.Subscriber('/lidar_points', Float32MultiArray, manage_lidar)

    last_lane_change_time = rospy.get_time()
    
    rate = rospy.Rate(10)
    rate.sleep()

def calculate_lookAhead_waypoint(odom):
    global current_position, yaw, path_flag

    # Update current position and orientation
    current_position[0] = odom.pose.pose.position.x
    current_position[1] = odom.pose.pose.position.y
    orientation_q = odom.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (_, _, yaw) = euler_from_quaternion(orientation_list)

    # Select the radius based on the path_flag
    if path_flag == 1:
        radius = radius_1
    else:
        radius = radius_2

    # Generate look-ahead points along the selected circular path
    angle = np.arctan2(current_position[1], current_position[0])
    lookAhead_angle = angle + lookAhead / radius
    lookAhead_point = (radius * np.cos(lookAhead_angle), radius * np.sin(lookAhead_angle))

    calculate_Curvature_nd_Steering(lookAhead_point)

def calculate_Curvature_nd_Steering(lookAhead_point):
    global yaw, current_position, wheel_base

    dx = lookAhead_point[0] - current_position[0]
    dy = lookAhead_point[1] - current_position[1]
    local_x = np.cos(yaw) * dx + np.sin(yaw) * dy
    local_y = -np.sin(yaw) * dx + np.cos(yaw) * dy

    if local_x == 0:
        control_line(0)
        return

    curvature = 2 * local_y / (local_x**2 + local_y**2)
    steering_angle = np.arctan(curvature * wheel_base) * 180 / np.pi

    if abs(steering_angle) < 0.1:  # TUNABLE
        steering_angle = 0

    control_line(steering_angle)

def control_line(steering):
    global steering_pub, cmd_pub, brake_pub, object_detected, TTC, x, y, last_lane_change_time

    rospy.loginfo(steering)
    steering_pub.publish(steering)
    
    # Check for human
    if object_detected == "person" and np.sqrt(x**2 + y**2) <= human_stop_distance:
        cmd_pub.publish(0)
        brake_pub.publish(1)
        rospy.sleep(3)
    # Check for lane change condition
    elif object_detected in ["car", "cone"] and np.sqrt(x**2 + y**2) <= lane_change_distance:
        current_time = rospy.get_time()
        if current_time - last_lane_change_time > lane_change_cooldown:
            change_lane()
            last_lane_change_time = current_time
    else:
        cmd_pub.publish(0.1)  # Constant speed
        brake_pub.publish(0)

    # Example stopping condition, you can adjust this based on your needs

def manage_object(msg):
    global object_detected
    object_detected = msg.data

def manage_lidar(msg):
    global TTC, x, y
    x = msg.data[0]
    y = msg.data[1]
    z = msg.data[2]
    distance = msg.data[3]
    relative_velocity = msg.data[4]
    TTC = msg.data[5]

def change_lane():
    global path_flag
    if path_flag == 1:
        path_flag = 2
    else:
        path_flag = 1

if __name__ == '__main__':
    try:
        init_node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
