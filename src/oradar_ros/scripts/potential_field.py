#!/usr/bin/env python3
import rospy
import numpy as np
import matplotlib.pyplot as plt
from sensor_msgs.msg import LaserScan
import time

# Setup the target point and the size of the grid
target = np.array([3, 3])
grid_size = np.array([3, 3])

# Initialize the potential field
potential_field = np.zeros((grid_size[0]*10, grid_size[1]*10))

# Constants for the attractive and repulsive fields
K_att = 1
K_rep = 100
d0 = 0.2  # Radius of influence for the obstacles

# Variable to keep track of the last plot time
last_plot_time = 0

def callback(data):
    global potential_field, last_plot_time

    # Calculate the attractive field towards the target
    for i in range(grid_size[0]*10):
        for j in range(grid_size[1]*10):
            point = np.array([i/10, j/10])
            dist_to_target = np.linalg.norm(target - point)
            potential_field[i, j] += K_att * dist_to_target**2

    # Calculate the repulsive field from the lidar data
    for angle_index in range(len(data.ranges)):
        angle = data.angle_min + angle_index * data.angle_increment
        dist = data.ranges[angle_index]
        if dist == np.inf or np.isnan(dist):
            continue

        # Calculate the position of the obstacle
        obstacle = np.array([dist*np.cos(angle), dist*np.sin(angle)]) * 10
        obstacle = np.round(obstacle).astype(int)

        # Add the repulsive field around the obstacle
        for i in range(max(0, obstacle[0]-10), min(grid_size[0]*10, obstacle[0]+10)):
            for j in range(max(0, obstacle[1]-10), min(grid_size[1]*10, obstacle[1]+10)):
                dist_to_obstacle = np.linalg.norm(obstacle - np.array([i, j]))
                if dist_to_obstacle <= d0:
                    potential_field[i, j] += K_rep * (1/dist_to_obstacle - 1/d0)**2

    # Plot the potential field every 30 seconds
    current_time = time.time()
    if current_time - last_plot_time > 30:
        plt.imshow(potential_field, origin='lower')
        plt.colorbar(label='Potential')
        plt.plot(target[1]*10, target[0]*10, 'ro')  # Target
        plt.show()
        last_plot_time = current_time

def listener():
    rospy.init_node('lidar_listener', anonymous=True)
    rospy.Subscriber("/MS200/scan", LaserScan, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()

