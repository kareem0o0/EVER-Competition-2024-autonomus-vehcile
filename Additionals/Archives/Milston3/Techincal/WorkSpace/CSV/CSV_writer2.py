import rospy
import csv
import math
import numpy as np
import time
from std_msgs.msg import Float64, Int32
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2
from geometry_msgs.msg import Point, Quaternion, Vector3
import sensor_msgs.point_cloud2 as pc

# Initialize global variables
start_time = time.time()
flag = 0
Dict = {}

x_odom_list = []
y_odom_list = []
yaw_odom_list = []
velocity_odom = []
acceleration_odom = []

angular_velocity_x_imu = []
angular_velocity_y_imu = []
angular_velocity_z_imu = []

# Calculate velocity and acceleration
def calculate_velocity_acceleration(prev_x, prev_y, prev_time, curr_x, curr_y, curr_time):
    velocity = math.sqrt((curr_x - prev_x)**2 + (curr_y - prev_y)**2) / (curr_time - prev_time)
    acceleration = (velocity - (math.sqrt((prev_x - prev_x)**2 + (prev_y - prev_y)**2) / prev_time)) / (curr_time - prev_time)
    return velocity, acceleration

# Calculate mean and RMS
def calculate_mean_rms(data_list):
    mean_value = np.mean(data_list)
    rms_value = np.sqrt(np.mean(np.square(data_list)))
    return mean_value, rms_value

# Odometry callback
def odom_callback(msg):
    global x_odom_list, y_odom_list, yaw_odom_list, velocity_odom, acceleration_odom, Dict, start_time
    x_odom = msg.pose.pose.position.x
    y_odom = msg.pose.pose.position.y
    yaw_odom = msg.pose.pose.orientation.z
    curr_time = time.time() - start_time

    if x_odom_list and y_odom_list:
        prev_x = x_odom_list[-1]
        prev_y = y_odom_list[-1]
        prev_time = curr_time - 1
        velocity, acceleration = calculate_velocity_acceleration(prev_x, prev_y, prev_time, x_odom, y_odom, curr_time)
        velocity_odom.append(velocity)
        acceleration_odom.append(acceleration)
    else:
        velocity_odom.append(0)
        acceleration_odom.append(0)

    x_odom_list.append(x_odom)
    y_odom_list.append(y_odom)
    yaw_odom_list.append(yaw_odom)

    mean_x_odom, rms_x_odom = calculate_mean_rms(x_odom_list)
    mean_y_odom, rms_y_odom = calculate_mean_rms(y_odom_list)
    mean_yaw_odom, rms_yaw_odom = calculate_mean_rms(yaw_odom_list)

    Dict.update({
        "positions_x_odom": x_odom,
        "positions_y_odom": y_odom,
        "velocity": velocity_odom[-1],
        "acceleration": acceleration_odom[-1],
        "yaw_odom": yaw_odom,
        "Time_Sec": curr_time,
        "mean_x_odom": mean_x_odom,
        "rms_x_odom": rms_x_odom,
        "mean_y_odom": mean_y_odom,
        "rms_y_odom": rms_y_odom,
        "mean_yaw_odom": mean_yaw_odom,
        "rms_yaw_odom": rms_yaw_odom
    })

    CSV_SAVE()

# IMU callback
def imu_callback(msg):
    global angular_velocity_x_imu, angular_velocity_y_imu, angular_velocity_z_imu, Dict
    angular_velocity_x = msg.angular_velocity.x
    angular_velocity_y = msg.angular_velocity.y
    angular_velocity_z = msg.angular_velocity.z
    angular_velocity_x_imu.append(angular_velocity_x)
    angular_velocity_y_imu.append(angular_velocity_y)
    angular_velocity_z_imu.append(angular_velocity_z)
    
    Dict.update({
        "angular_velocity_x_imu": angular_velocity_x,
        "angular_velocity_y_imu": angular_velocity_y,
        "angular_velocity_z_imu": angular_velocity_z
    })
    
    CSV_SAVE()

# Command velocity callback
def cmd_vel_callback(msg):
    velocity_cmd = msg.data
    Dict.update({"velocity_cmd": velocity_cmd})
    CSV_SAVE()

# Steering angle callback
def steering_angle_callback(msg):
    steering_angle = msg.data
    Dict.update({"steering_angle": steering_angle})
    CSV_SAVE()


#------------------------------------------------------------------
#             ADD Ridar Reading
#-------------------------------------------------------------------
# Point cloud callback (commented out)
# def point_cloud_callback(msg):
#     x_points = []
#     y_points = []
#     z_points = []
#     for point in pc.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
#         x_points.append(point[0])
#         y_points.append(point[1])
#         z_points.append(point[2])
#     Dict.update({"x_points": x_points, "y_points": y_points, "z_points": z_points})
#     CSV_SAVE()

def listener():
    rospy.init_node('listen', anonymous=True)
    rospy.Subscriber("odom", Odometry, odom_callback)
    rospy.Subscriber("imu", Imu, imu_callback)
    rospy.Subscriber("cmd_vel", Float64, cmd_vel_callback)
    rospy.Subscriber("SteeringAngle", Float64, steering_angle_callback)
    
    # subscriber of lidar
    # rospy.Subscriber("velodyne_points", PointCloud2, point_cloud_callback)
    rospy.spin()

def CSV_SAVE():
    global flag
    with open("/home/eslam/catkin_workspace/src/t2/scripts/Dimensions31.csv", mode="a") as csvfile:
        fieldnames = [
            "positions_x_odom", "positions_y_odom", "velocity", "acceleration", 
            "yaw_odom", "Time_Sec", "angular_velocity_x_imu", "angular_velocity_y_imu", 
            "angular_velocity_z_imu", "velocity_cmd", "steering_angle",
            "mean_x_odom", "rms_x_odom", "mean_y_odom", "rms_y_odom", "mean_yaw_odom", "rms_yaw_odom"
            # Delecrtion of lidar point
            #, "x_points", "y_points", "z_points"
        ]
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        if flag == 0:
            writer.writeheader()
            flag = 1
        writer.writerow(Dict)

if __name__ == '__main__':
    listener()
