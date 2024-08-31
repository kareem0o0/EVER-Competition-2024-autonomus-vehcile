#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Int32
import time
import math as m

angle =0

def control():

    rospy.init_node ('control', anonymous=True)

    pub_SteeringAngle = rospy.Publisher('SteeringAngle', Float64, queue_size=10)
    pub_cmd_vel = rospy.Publisher('cmd_vel', Float64, queue_size=10)
    pub_brakes = rospy.Publisher('brakes', Float64, queue_size=10)

    start = time.time()
    while (time.time()- start) <20:

        if ((time.time() - start) <= 8) :
            #curve
            pub_SteeringAngle.publish(18.5)
            pub_cmd_vel.publish(0.23)
            pub_brakes.publish(0)

        elif ((time.time() - start) > 8) and ((time.time() - start) <= 11) :
            #straight
            pub_SteeringAngle.publish(0)
            pub_cmd_vel.publish(0.23)
            pub_SteeringAngle.publish(0)
            pub_brakes.publish(0)

        elif ((time.time() - start) > 11) and ((time.time() - start) <= 16.9) :
            #curve
            pub_SteeringAngle.publish(-18.5)
            pub_cmd_vel.publish(0.23)
            pub_brakes.publish(0)

        elif ((time.time() - start) >16.9) and ((time.time() - start) <=18):
            #straight
            pub_SteeringAngle.publish(0)
            pub_cmd_vel.publish(0)

if __name__ == '__main__':
    try:
        control()
    except rospy.ROSInterruptException:
        pass
