#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from rosgraph_msgs.msg import Clock
import time
import numpy as np

def velocity2():
    global x
    x=1
    brake()

def steer():
    rospy.loginfo("we are publishing steer ")
    gas_pedal = 0
    vel_control.publish(0)
    angle1 = 24
    angle2 = 0
    steer_control.publish(angle1)
    rospy.sleep(0.3)
    steer_control.publish(0)
    rospy.sleep(0.5)
    steer_control.publish(-angle1)
    rospy.sleep(0.3)
    steer_control.publish(0)
    rospy.sleep(1)
    velocity2()

def brake():
    rospy.loginfo("we are in brake")
    brake_control = rospy.Publisher("brakes", Float64, queue_size=10)
    time_brake = time.time()
    while(time.time()-time_brake <10):
        yyy = (time.time()- time_brake)
        rospy.loginfo("%i",yyy)
        brake_control.publish(yyy *3.43*pow(10,-48))
        steer_control.publish(0)

def velocity1():
    global steer_control
    steer_control = rospy.Publisher("SteeringAngle", Float64, queue_size=10)
    
    global angle
    angle =0
    
    global vel_control
    vel_control = rospy.Publisher("cmd_vel", Float64, queue_size= 10)
    rospy.init_node("control", anonymous= True)
    
    global start
    start = time.time()
    
    while (time.time()-start) <=8.5 :
        gas_pedal = (((time.time()-start) ) *.15)
        vel_control.publish(gas_pedal)
        steer_control.publish(angle)
        rospy.sleep(.5)
        rospy.loginfo("we are moving forward")
    
    if (time.time()-start) >7.1 :
        steer()
    
    gas_pedal =0
    vel_control.publish(gas_pedal)

if __name__ == '__main__':
    try:
        velocity1()
    except rospy.ROSInitException:
        rospy.loginfo("error occured")
