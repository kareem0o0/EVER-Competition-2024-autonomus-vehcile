#!/usr/bin/env python3


import pandas as pd
import matplotlib.pyplot as plt

# Read the CSV file
df = pd.read_csv('/home/daino/workspace/src/real_time/scripts/wp_file.csv')

# Create the plot
plt.figure(figsize=(12, 10))
plt.plot(df['positions_y_odom'], df['positions_x_odom'], '-b', linewidth=0.5)
plt.scatter(df['positions_y_odom'], df['positions_x_odom'], c=range(len(df)), cmap='viridis', s=2)

# Set labels and title
plt.xlabel('Y Position')
plt.ylabel('X Position')
plt.title('Odometry Path')

# Add colorbar to show progression
cbar = plt.colorbar()
cbar.set_label('Time Progression')

# Set aspect ratio to equal for accurate representation
plt.axis('equal')

# Add grid
plt.grid(True, linestyle='--', alpha=0.7)

# Show the plot
plt.tight_layout()
plt.show()