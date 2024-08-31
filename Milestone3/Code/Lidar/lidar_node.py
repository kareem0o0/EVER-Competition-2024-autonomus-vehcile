#!/usr/bin/env python

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32MultiArray
import numpy as np
import math
import time

# Initialize the counter
counter = 0

# Initialize the publisher
pub = rospy.Publisher('lidar_points', Float32MultiArray, queue_size=10)

# Initialize global variables to store previous distance and time
prev_distance = None
prev_time = None
relative_velocity = 100
TTC = 50
distance = 50

def point_cloud_callback(msg):
    global counter, prev_distance, prev_time, relative_velocity, TTC, distance

    # Read points from the PointCloud2 message
    points = np.array(list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)))

    # Parameters for filtering and clustering
    target_height = 0.1  # 0.5 meters below the LiDAR
    height_tolerance = 0.2  # Tolerance for height filtering
    x_range = (-10, 10)  # Range of x coordinates
    y_range = (0, 14)  # Range of y coordinates
    clustering_distance = 2.0  # Maximum distance for clustering points together

    # Filtered and clustered points initialization
    filtered_points = []
    clustered_points = []

    # Filter points by height and position
    for point in points:
        x, y, z = point[:3]

        # Check if the point is within the height tolerance
        if abs(z - target_height) <= height_tolerance:
            # Check if the point is within the x, y range
            if x_range[0] <= x <= x_range[1] and y_range[0] <= y <= y_range[1]:
                filtered_points.append((x, y, z))

    # Convert filtered points to numpy array for clustering
    filtered_points = np.array(filtered_points)

    # Perform clustering
    while len(filtered_points) > 0:
        # Initialize cluster with the first point
        cluster = [filtered_points[0]]
        remaining_points = []

        # Find nearby points and form clusters
        for point in filtered_points[1:]:
            dist = np.linalg.norm(np.array(cluster)[:, :2] - point[:2], axis=1)
            if np.min(dist) <= clustering_distance:
                cluster.append(point)
            else:
                remaining_points.append(point)

        # Compute the centroid of the cluster
        cluster = np.array(cluster)
        centroid = np.mean(cluster, axis=0)
        clustered_points.append(centroid)

        # Update filtered points to remaining points
        filtered_points = np.array(remaining_points)

    # Convert clustered points to numpy array for further processing
    clustered_points = np.array(clustered_points)
    counter += 1

    # Check if there are no clustered points
    if len(clustered_points) == 0:
        # Publish default values
        pub.publish(Float32MultiArray(data=[100] * 6))
    else:
        # Process the clustered points to calculate velocity
        for point in clustered_points:
            x, y, z = point
            rospy.loginfo(f"({counter}) Clustered Point: x={x}, y={y + 0.1}, z={z - 1.54}")  # With position offset to the vehicle origin

            # Measure the horizontal distance from the vehicle (neglecting Z)
            distance = math.sqrt(x**2 + y**2)

            # Get the current time
            current_time = time.time()

            # If we have a previous measurement, calculate the velocity
            if prev_distance is not None and prev_time is not None:
                # Calculate the change in distance
                delta_distance = distance - prev_distance

                # Calculate the change in time
                delta_time = current_time - prev_time

                # Avoid division by zero
                if delta_time > 0:
                    # Calculate the velocity
                    relative_velocity = delta_distance / delta_time

            # Update previous distance and time
            prev_distance = distance
            prev_time = current_time
            if relative_velocity > 0:
                TTC = distance / relative_velocity
            # Publish the results
            pub.publish(Float32MultiArray(data=[x, y, z, distance, relative_velocity, TTC]))  # Keep track of the arrangement of data

def main():
    rospy.init_node('clustered_point_lidar', anonymous=True)
    rospy.Subscriber('/velodyne_points', PointCloud2, point_cloud_callback, queue_size=1)
    rospy.spin()

if __name__ == '__main__':
    main()
