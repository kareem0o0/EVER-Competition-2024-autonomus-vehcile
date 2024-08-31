#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float32, Float64
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

y_values = np.linspace(-2.375, 2.375, 15)
x_values = np.linspace(-3.832, 3.832, 15)
right_to_left = list(zip(x_values, y_values))

y_values = np.linspace(2.375, -2.375, 15)
left_to_right = list(zip(x_values, y_values))

radius = 7.275

def init_node():

    global cmd_pub, steering_pub, brake_pub

    rospy.init_node("pure_pursuit_control", anonymous=False)

    rospy.Subscriber('/aft_mapped_adjusted', Odometry, callvack)

    cmd_pub = rospy.Publisher("/in_Car_velocity_in_KM/H", Float32, queue_size=10) 

    steering_pub = rospy.Publisher("/in_Car_steering_in_degree", Float32, queue_size=10)

    rate = rospy.Rate(10)
    rate.sleep()


look_ahead1 = 1
look_ahead2 = 2
wheel_base_real= 1.18
wheel_base = 2.26963


def callvack(odom):
    global C_pose, yaw

    C_pose = [0.0,0.0]
    C_pose[0] = odom.pose.pose.position.x
    C_pose[1] = odom.pose.pose.position.y
    orientation_q = odom.pose.pose.orientation
    z_orien = odom.pose.pose.orientation.z
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (_, _, yaw) = euler_from_quaternion(orientation_list)

    if C_pose[1] >= -2.375 and C_pose[1] <= 2.375 and yaw <=0:
        rospy.loginfo("left to right")
        waypoints = left_to_right
        for point in waypoints:
            if point[1] < (C_pose[1] - look_ahead1) and point[1] >= (C_pose[1] - look_ahead1-2):
                calculate_curv(point)
                return

    if C_pose[1]>= -2.375 and C_pose[1]<= 2.375 and yaw > 0:
        rospy.loginfo("right to left")

        waypoints = right_to_left
        for point in waypoints:
            if point[1] > (C_pose[1] + look_ahead1) and point[1] <= (C_pose[1] + look_ahead1+2):
                calculate_curv(point)
                return
    

    if C_pose[1] > 2.375 :
        rospy.loginfo("left circle")

        cy = 8.56
        cx = 0
        angle = np.arctan2(C_pose[1]-cy, C_pose[0]-cx)
        lookAhead_angle = angle + look_ahead2 / radius
        lookAhead_point = (cx+(radius * np.cos(lookAhead_angle)), cy+(radius * np.sin(lookAhead_angle)))
        calculate_curv(lookAhead_point)
    
    if C_pose[1] < -2.375 :
        rospy.loginfo("right circle")

        cy = - 8.56
        cx = 0
        angle = np.arctan2(C_pose[1]-cy, C_pose[0]-cx)
        lookAhead_angle = angle - look_ahead2 / radius
        lookAhead_point = (cx+(radius * np.cos(lookAhead_angle)), (radius * np.sin(lookAhead_angle))+cy)

        calculate_curv(lookAhead_point)


def calculate_curv(point):
   # Calculate relative position 
    dx = point[0] - C_pose[0]
    dy = point[1] - C_pose[1]

    # Transform to local coordinates
    cos_yaw = np.cos(yaw)
    sin_yaw = np.sin(yaw)
    local_x = cos_yaw * dx +sin_yaw * dy
    local_y = -sin_yaw * dx + cos_yaw * dy
    
    # Avoid division by zero or very small numbers
    if abs(local_x) < 1e-6:
        local_x = 1e-6 if local_x >= 0 else -1e-6
    
    # Calculate curvature
    curvature = 2.0 * local_y  / (local_x**2 + local_y**2)
    

    min_curvature= .395
    max_steering_angle= 20
    # Limit curvature to avoid extreme values
    curvature = np.clip(curvature, -min_curvature, min_curvature)
    
    # Calculate steering angle
    steering_angle = np.arctan2(wheel_base * curvature, 1.0)
    
    # Convert to degrees and limit the steering angle
    steering_angle_deg = np.degrees(steering_angle)
    steering_angle_deg = np.clip(steering_angle_deg, -max_steering_angle, max_steering_angle)
    
    # Apply a small deadband to reduce jitter
    if abs(steering_angle_deg) < 3:
        steering_angle_deg = 0
    steering_angle_deg *=-1
    rospy.loginfo(steering_angle_deg)
    steering_pub.publish(steering_angle_deg)
    velocity = Float32()
    int_steering = int(steering_angle_deg)
    velocity.data= 5 * ( 1 - (int_steering/25) )
    cmd_pub.publish(velocity)



if __name__ == '__main__':
    try:



        init_node()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass