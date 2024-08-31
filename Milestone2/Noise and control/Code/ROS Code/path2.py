#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Float64, Int16, Bool, Int32, Float32MultiArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import time
import math

start = time.time()
# d = 37.714   2*pi*r         d = 0.5*a*t^2

class MyNode:
    def __init__(self):
        rospy.init_node('path2_node', anonymous=True)
        self.pub = rospy.Publisher('SteeringAngle', Float64, queue_size=10)
        self.pub2 = rospy.Publisher('cmd_vel', Float64, queue_size=10)
        self.pub3 = rospy.Publisher('brakes', Float64, queue_size=10)
        self.pub4 = rospy.Publisher('startSimulation', Bool, queue_size=10)
        self.pub5 = rospy.Publisher('pauseSimulation', Bool, queue_size=10)
                          
        
         
        self.error_sum = 0
        self.last_error = 0

        self.Kp = 0.1
        self.Ki = 0
        self.Kd = 8
        self.desired_radius = 6
        self.error = 0
        self.integral = 0
        self.derivative = 0
        self.last_error = 0
        self.desired_steering_angle = 18.5 
        self.last_time = rospy.get_time()

    def compute_pid(self):

        current_time = rospy.get_time()
        dt = current_time - self.last_time if self.last_time != 0 else 0.1
        self.error = self.desired_radius - C
        self.integral += self.error * dt
        self.derivative = (self.error - self.last_error) / dt if dt > 0 else 0

        self.steering_angle = self.desired_steering_angle + self.Kp * self.error + self.Ki * self.integral + self.Kd * self.derivative
        self.last_error = self.error
        self.last_time = current_time

        return self.steering_angle
    
    def odom_callback(self, point):

        global xpose, ypose, yaw,C,state
        
        xpose = point.data[0]
        ypose = point.data[1]
        yaw = point.data[2]
        

        C = math.sqrt((xpose+6)**2 + ypose**2)    #desired = 6
        state = xpose**2 + ypose**2                  

        rospy.loginfo("x position =  {}".format(xpose))
        rospy.loginfo("y position =  {}".format(ypose))
        rospy.loginfo("yaw =  {}".format(yaw))
        
        rospy.loginfo("c =  {}".format(state))

        new_steering_angle = self.compute_pid()

        self.pub.publish(new_steering_angle)
        
        if state < 150 and ypose<-4 and ypose>-5:                #if x > and y < -1 and y>-2:
            gaspedal = 0
            self.pub2.publish(gaspedal)
            brakepedal= (time.time() - start) * 1.37* pow(10, -48) 
            self.pub3.publish(brakepedal) 
            rospy.signal_shutdown("Condition met")   

    def od_subscriber(self):
        rospy.Subscriber("/odom1", Float32MultiArray, self.odom_callback)

    def run(self):
        rate = rospy.Rate(1)  # 1 Hz
    
        while not rospy.is_shutdown():
                
            steering_angle = 18.5  # Change this value as needed
            gaspedal = 0.23
            brakepedal = 0
            startsim = True

            #rospy.loginfo("Publishing: {}".format(data_to_publish))
            self.pub4.publish(Bool(startsim))
            self.pub.publish(steering_angle)
            self.pub2.publish(gaspedal)
            self.pub3.publish(brakepedal)
            my_node.od_subscriber()
                
            rate.sleep()

 
if __name__== '__main__':
    my_node = MyNode()
    try:
        my_node.run()
    except rospy.ROSInterruptException:
        pass