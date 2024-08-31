#!/usr/bin/env python

import rospy
import csv
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from std_msgs.msg import Int32
import time


#X = float()
#Y = float()
flag = 0

start = time.time()



def position(POSE):
    global X , Y, Dict, Timer
    msg = POSE
    X = msg.pose.pose.position.x
    Y = msg.pose.pose.position.y
    Dict = {"positions_X": X, "positions_y": Y, "Time_Sec": (time.time()- start)}
    CSV_SAVE()

def listener1():
    rospy.init_node('listen', anonymous=True)
    rospy.Subscriber("odom", Odometry, position)
    rospy.spin()

def CSV_SAVE():
    global flag
    rate = rospy.Rate(20) # 1hz
    with open("/home/daino/catkin_workspace/src/milestone1/scripts/Dimensions.csv", mode="a") as csvfile:
        fieldnames = ["positions_X", "positions_y", "Time_Sec"]
        writer = csv.DictWriter(csvfile, fieldnames = fieldnames)
        if flag == 0:
            writer.writeheader()
            flag = 1
        writer.writerow(Dict)
        rate.sleep()


if __name__ == '__main__':
    listener1()