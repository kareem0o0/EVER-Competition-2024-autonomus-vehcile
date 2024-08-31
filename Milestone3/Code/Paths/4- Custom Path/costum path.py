#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Float64, String, Float32MultiArray

# Global parameters
wheel_base = 2.26963
lookAhead = 2
current_position = [0.0, 0.0]
circle_flag = 0
flag = "L1"  # Initial flag
human_stop_distance = 14  # Distance to stop for a human
lane_change_distance = 11  # Distance to change lane for a car or cone
lane_change_cooldown = 8  # Cooldown period for lane changes (seconds)
speed = 0.15  # Constant speed
full_path = []
detected_object = ""
x = y = 0
last_lane_change_time = 0




'''def flag_control(detected_object="car", TTC=5):
    global flag

    Tmin = 4

    if detected_object == "person":
        emergency_stop()
    elif (detected_object in ["car", "cone"]) and TTC >= Tmin:
        flag = "L1" if flag == "L2" else "L2"
    elif (detected_object in ["car", "cone"]) and TTC < Tmin:
        emergency_stop()'''

def init_node():
    global cmd_pub, steering_pub, brake_pub

    rospy.init_node("pure_pursuit_control", anonymous=True)

    rospy.Subscriber('/odom', Odometry, calculate_lookAhead_waypoint)

    cmd_pub = rospy.Publisher('/cmd_vel', Float64, queue_size=10)
    steering_pub = rospy.Publisher('/SteeringAngle', Float64, queue_size=10)
    brake_pub = rospy.Publisher('/brakes', Float64, queue_size=10)
    rospy.Subscriber('/detected_labels', String, manage_object)
    rospy.Subscriber('/lidar_points', Float32MultiArray, manage_lidar)

    rate = rospy.Rate(10)
    rate.sleep()

def calculate_lookAhead_waypoint(odom):
    global current_position, yaw,flag

    # Update current position and orientation
    current_position[0] = odom.pose.pose.position.x
    current_position[1] = odom.pose.pose.position.y
    orientation_q = odom.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (_, _, yaw) = euler_from_quaternion(orientation_list)
    flag_control()
    rospy.loginfo(flag)



    if current_position[0] < 50 and current_position[0] >0 and current_position[1]<20 :
        rospy.loginfo("up")
        waypoints = upper_path2 if flag == "L2" else  U_path
        for point in waypoints:
            if point[0] > (current_position[0] + lookAhead) and point[0] <= (current_position[0] + lookAhead+3):
                calculate_curv_s(point)
                return


    elif current_position[0] >50 :
        rospy.loginfo("left")
       
        radius = 24 if flag == "L2" else  20


        angle = np.arctan2(current_position[1], current_position[0]-50)
        lookAhead_angle = angle + lookAhead / radius
        lookAhead_point = (50+(radius * np.cos(lookAhead_angle)), 20+(radius * np.sin(lookAhead_angle)))
        rospy.loginfo(lookAhead_point)
        calculate_curv_s(lookAhead_point)


    elif current_position[0]<= 50 and current_position[0] > 0 and current_position[1]>20:
        rospy.loginfo("down")
        waypoints = lower_path2 if flag == "L2" else  L_path
        for point in waypoints:
            if point[0] < (current_position[0] - lookAhead) and point[0] >= (current_position[0] - lookAhead-3):
                calculate_curv_s(point)
                return


    elif current_position[0]< 0 :  
        radius = 24 if flag == "L2" else  20
        rospy.loginfo("right")
        angle = np.arctan2(current_position[1]+200, current_position[0])
        lookAhead_angle = angle + lookAhead / radius
        lookAhead_point = (-(radius * np.cos(lookAhead_angle)), (radius * np.sin(lookAhead_angle))-20)
        calculate_curv_s(lookAhead_point)





def calculate_curv_s(point):
    dy = point[1] - current_position[1]
    dx = current_position[0] - point[0]
    local_x = np.cos(yaw) * dy + np.sin(yaw) * dx
    local_y = -np.sin(yaw) * dy + np.cos(yaw) * dx

    curvature = 2 * local_y / (local_x ** 2 + local_y ** 2)
    steering_angle = np.arctan(curvature * wheel_base) * 180 / np.pi
    if abs(steering_angle) <= .5:
       steering_angle = 0

  #  rospy.loginfo(steering_angle)
    steering_pub.publish(Float64(steering_angle))
    cmd_pub.publish(Float64(.2))

def manage_object(msg):
    global detected_object
    detected_object = msg.data

def manage_lidar(msg):
    global x, y
    x = msg.data[0]
    y = msg.data[1]

def flag_control():
    global detected_object, x, y, flag, last_lane_change_time

    if detected_object == "person" :#and y < human_stop_distance and abs(x) < 7:  # TUNABLE
        emergency_stop()
    elif detected_object in ["car", "cone"]:
        current_time = rospy.get_time()
        if np.sqrt(x**2 + y**2) <= lane_change_distance and current_time - last_lane_change_time > lane_change_cooldown:  # TUNABLE
            change_lane()
            last_lane_change_time = current_time


def change_lane():
    global flag
    if flag == "L1":
        flag = "L2"
    else:
        flag = "L1"

def emergency_stop():
    cmd_pub.publish(0)
    #steering_pub.publish(0)
    brake_pub.publish(1)
    rospy.sleep(3)
    brake_pub.publish(0)



if __name__ == '__main__':
    try:
        lookAhead = 3

        # Generate the upper and lower straight line
        x_upper = np.linspace(0, 50, 100)
        y_upper = x_upper * 0
        U_path = list(zip(x_upper, y_upper))
        
        x_lower = x_upper
        y_lower = y_upper
        y_lower[:] = 40
        L_path = list(zip(x_lower, y_lower))

        # Generate the first half circle
        theta1 = np.linspace(0, np.pi, 100)
        radius = 20
        x_circle1 =  radius * np.cos(theta1)
        y_circle1 =20 - radius * np.sin(theta1)
        R_circle = list(zip(x_circle1, y_circle1))

        # Generate the left half circle
        theta2 = np.linspace(0, np.pi , 100)
        radius = 20
        x_circle2 = 50 + radius * np.cos(theta2)
        y_circle2 = 20 + radius * np.sin(theta2)
        L_circle = list(zip(x_circle2, y_circle2))




        #-----------------------------------------------

        #lane 2
        y_upper[:] = -4
        upper_path2 = list(zip(x_upper, y_upper))

        y_lower [:] =  44
        lower_path2 = list(zip(x_lower, y_lower))       


        # Combine all paths
        full_path = U_path + L_circle + L_path + R_circle

        init_node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass