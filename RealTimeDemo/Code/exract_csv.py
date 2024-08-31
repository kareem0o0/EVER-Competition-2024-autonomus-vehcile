#!/usr/bin/env python


import rosbag
import csv

# Define the bag file and output CSV file
bag_file = '/home/daino/Desktop/2024-08-14-19-23-49.bag'
output_csv = '/home/daino/Desktop/final.csv'

# Open the bag file
with rosbag.Bag(bag_file, 'r') as bag:
    with open(output_csv, 'w') as csvfile:
        writer = csv.writer(csvfile)
        # Write header (adjust according to your message structure)
        writer.writerow(['xcolumn', 'ycolumn'])
        
        # Iterate through the bag file
        for topic, msg, t in bag.read_messages(topics=['/']):
            # Extract data from the message and write to CSV
            writer.writerow([t.to_sec(), msg.field1, msg.field2, '...'])
