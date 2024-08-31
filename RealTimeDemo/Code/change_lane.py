#!/usr/bin/env python


#---------------------------------------------------------------------------------------------------------------------------

#   THERE ARE SOME IMPORTANT NOTES AFTER THE CODE ENDS, READ THEM !!!!!!!!!!!!!!
import rospy
import numpy as np
from std_msgs.msg import Float32, Float64
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


def init_node():

    global pub_vel, pub_steering

    rospy.init_node("pure_pursuit_control", anonymous=False)


    rospy.Subscriber('/aft_mapped_adjusted', Odometry, callvack)
    pub_vel = rospy.Publisher('/in_Car_velocity_in_KM/H', Float32, queue_size=10)
    pub_steering = rospy.Publisher('/in_Car_steering_in_degree', Float32, queue_size=10)

    rate = rospy.Rate(10)
    rate.sleep()


look_ahead1 = 4
look_ahead2 = 0.9
wheel_base_real= 1.18
wheel_base_real = 1.18
 

def callvack(odom):
    global C_pose, yaw

    C_pose = [0.0,0.0]
    C_pose[0] = odom.pose.pose.position.x
    C_pose[1] = odom.pose.pose.position.y
    orientation_q = odom.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (_, _, yaw) = euler_from_quaternion(orientation_list)

    if C_pose[0] <7 :
        rospy.loginfo(C_pose)
        rospy.loginfo("first")
        waypoints  = O
            


    elif C_pose[0] >7:
        rospy.loginfo("Second")
        rospy.loginfo(C_pose)
        waypoints = S



    for point in waypoints:
        if point[0] > (C_pose[0] + look_ahead1) and point[0] <= (C_pose[0] + look_ahead1 + 3):
            calculate_curv(point)
            return





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
    max_steering_angle= 25
    # Limit curvature to avoid extreme values
    curvature = np.clip(curvature, -min_curvature, min_curvature)
    
    # Calculate steering angle
    steering_angle = np.arctan2(wheel_base_real * curvature, 1.0)
    
    # Convert to degrees and limit the steering angle
    steering_angle_deg = np.degrees(steering_angle)
    steering_angle_deg = np.clip(steering_angle_deg, -max_steering_angle, max_steering_angle)
    
    # Apply a small deadband to reduce jitter
    if abs(steering_angle_deg) < 3:
        steering_angle_deg = 0

    #rospy.loginfo(f"local x is : {local_x} and local y is {local_y} ")
    steering_angle_deg *=-1
    rospy.loginfo(steering_angle_deg)
    pub_steering.publish(steering_angle_deg)
    velocity = Float32()
    velocity.data= 1.0
    pub_vel.publish(velocity)



if __name__ == '__main__':
    try:

        x_values = np.linspace(0,40, 80)
        y_values = x_values *0
        O = list(zip(x_values, y_values))
        y_values[:] = 4
        S = list(zip(x_values, y_values))

        radius = 7.275

        init_node()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass



# this code is valid on the simulation car , I fyou need to make itfir the real car the you have to :

#   1- change anonymous to false 
#   2- change the topics 
#   3- put some prints to know which meter you at 
#   4- keep distances tall to not face steering issues 
#
#
