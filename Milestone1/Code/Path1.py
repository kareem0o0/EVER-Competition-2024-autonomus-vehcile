#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from rosgraph_msgs.msg import Clock
import time
import numpy as np

def brake():
    brake_control = rospy.Publisher("brakes", Float64, queue_size=10)
    
    time_brake = time.time()
    while (time.time()-start) >7.8 and (time.time()-start) <9 :
        brake_pedal = 1         # (time.time()- time_brake) *1.37* pow(10, -47)
        brake_control.publish(brake_pedal)
        steer_control.publish(angle)

def velocity():
    global steer_control
    steer_control = rospy.Publisher("SteeringAngle", Float64, queue_size=10)
    
    global angle
    angle = 0
    
    vel_control = rospy.Publisher("cmd_vel", Float64, queue_size= 10)
    rospy.init_node("control", anonymous= True)
    
    global start
    start = time.time()
    
    while (time.time()-start) <=7.8:
        gas_pedal = (((time.time()-start) ) *.15)
        vel_control.publish(gas_pedal)
        steer_control.publish(angle)
    
    gas_pedal =0
    vel_control.publish(gas_pedal)
    
    brake()

if __name__ == '__main__':
    try:
        velocity()
    except rospy.ROSInitException:
        rospy.loginfo("error occured")
