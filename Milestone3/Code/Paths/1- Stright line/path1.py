#!/usr/bin/env python

import rospy 
import numpy as np
from std_msgs.msg import Float64, String, Float32MultiArray
from tf.transformations import euler_from_quaternion

# Global variables
global wheel_base 
global lookAhead
global current_position
global maxWheelVelocity 
global object_detected
global TTC, x

maxWheelVelocity = 114.3202437
TTC = 0
x = 0

wheel_base = 2.26963
lookAhead = 2 # TUNABLE
current_position = [0, 0]

def init_node():
    global cmd_pub, steering_pub, brake_pub

    rospy.init_node("straight_line_control", anonymous=True)
    cmd_pub = rospy.Publisher('/cmd_vel', Float64, queue_size=10)
    steering_pub = rospy.Publisher('/SteeringAngle', Float64, queue_size=10)
    brake_pub = rospy.Publisher('/brakes', Float64, queue_size=10)
    object_sub = rospy.Subscriber('/detected_labels', String, manage_object)
    lidar_sub = rospy.Subscriber('/lidar_points', Float32MultiArray, manage_lidar)
    control_line(0)
    rate = rospy.Rate(10)
    rate.sleep()

def control_line(steering):
    global steering_pub, cmd_pub, brake_pub, TTC, x

    steering_pub.publish(steering)
    cmd_pub.publish(0.267)
    
    while True:
        if TTC < 14 and abs(x) < 3: # TUNABLE
            steering_pub.publish(0)
            cmd_pub.publish(0)
            brake_pub.publish(1)
        else:
            steering_pub.publish(0)
            cmd_pub.publish(0.1)
            brake_pub.publish(0)

def manage_object(msg):
    global object_detected
    object_detected = msg

def manage_lidar(msg):
    global TTC, x
    x = msg.data[0]
    y = msg.data[1]
    z = msg.data[2]
    distance = msg.data[3]
    relative_velocity = msg.data[4]
    TTC = msg.data[5]

if __name__ == '__main__':
    try:
        init_node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
