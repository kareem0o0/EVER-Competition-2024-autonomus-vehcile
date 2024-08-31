#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Int32
import time

start = time.time()

def control():

    rospy.init_node ('control', anonymous=True)

    pub_SteeringAngle = rospy.Publisher('SteeringAngle', Float64, queue_size=10)
    pub_cmd_vel = rospy.Publisher('cmd_vel', Float64, queue_size=10)
    pub_brakes = rospy.Publisher('brakes', Float64, queue_size=10)

    while not rospy.is_shutdown():

        if ((time.time() - start) <=9 ):
            #rospy.loginfo("The car moving Forward")
            pub_SteeringAngle.publish(18.5)
            pub_cmd_vel.publish(0.23)
            pub_brakes.publish(0)

        elif ((time.time() - start) >9 ) and ((time.time() - start) <=9.5):
            pub_cmd_vel.publish(0)
            pub_brakes.publish(1)

        else:
            break

if __name__ == '__main__':
    try:
        control()
    except rospy.ROSInterruptException:
        pass
