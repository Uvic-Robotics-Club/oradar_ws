#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
import numpy as np
import math

# Set up plot
fig, ax = plt.subplots()

# Flag to enable or disable plotting
PLOT_ENABLED_distance = False
PLOT_ENABLED_SCORE = True
ROBOT_WIDTH = .4
LIDAR_FOV = 360
LIDAR_RESOLUTION = .8  # I found this by knowing 360 degrees/approx 450 data points per 1 rotation
LIDAR_MAX_RANGE = 12  # meters  # Maximum reliable range of the LIDAR

# Position of the robot. You'll need a way to keep track of this.
robot_position = [0, 0]  # [x, y]

# Position of the goal
goal_position = [3, 3]  # [x, y]

# Calculate the number of Lidar indices that represent the robot's width
ROBOT_WIDTH_INDICES = int((ROBOT_WIDTH / (2 * np.pi * LIDAR_MAX_RANGE)) * LIDAR_FOV / LIDAR_RESOLUTION)

def plot_data(data):
    global ax
    plt.cla()  # Clear the old plot
    ax.bar(range(len(data.ranges)), data.ranges)  # Create a new plot
    plt.draw()  # Update the plot
    plt.pause(0.001)  # Needed to update the plot

def plot_scores(scores):
    global ax
    plt.cla()  # Clear the old plot
    angles = np.linspace(0, LIDAR_FOV, len(scores))  # Calculate the corresponding angle for each score
    ax.plot(angles, scores)  # Create a new plot
    ax.set_xlabel('Angle (degrees)')  # Set x-axis label
    ax.set_ylabel('Score')  # Set y-axis label
    plt.draw()  # Update the plot
    plt.pause(0.001)  # Needed to update the plot

def callback(data):
    global PLOT_ENABLED_distance
    global PLOT_ENABLED_SCORE
    global robot_position
    global goal_position
    # Get the current time
    current_time = rospy.get_time()
    # Check if one second has passed since the last update
    if PLOT_ENABLED_distance and current_time - callback.last_update_time >= 1.0:
        plot_data(data)
        # Update the last update time
        callback.last_update_time = current_time

    # Calculate free space considering the robot's width
    free_space = [min(data.ranges[i:i+ROBOT_WIDTH_INDICES]) for i in range(len(data.ranges) - ROBOT_WIDTH_INDICES + 1)]

    # Calculate the goal direction
    goal_direction_dx = goal_position[0] - robot_position[0]
    goal_direction_dy = goal_position[1] - robot_position[1]
    goal_direction = math.atan2(goal_direction_dy, goal_direction_dx)

    # Calculate a score for each direction
    scores = []
    for i, distance in enumerate(free_space):
        if distance > ROBOT_WIDTH:
            # The angle difference to the target direction
            angle_difference = abs(i - len(free_space) // 2 - goal_direction)
            # The score is higher for larger distances and smaller angle differences
            score = distance / (1 + angle_difference)
            scores.append(score)
        else:
            scores.append(0)  # If there is not enough space for the robot, the score is 0

    # Find the direction with the highest score
    best_direction = scores.index(max(scores))

    if PLOT_ENABLED_SCORE and current_time - callback.last_update_time >= 1.0:
        plot_scores(scores)
        # Update the last update time
        callback.last_update_time = current_time

    # TODO: Command the robot to move in the best direction
    # For the purpose of this example, let's just move the robot a small step in the best direction
    step_size = 0.1  # The distance the robot moves in each step
    robot_position[0] += step_size * math.cos(best_direction)
    robot_position[1] += step_size * math.sin(best_direction)

def listener():
    rospy.init_node('lidar_plot', anonymous=True)
    # Initialize the last update time to the current time
    callback.last_update_time = rospy.get_time()
    rospy.Subscriber("/MS200/scan", LaserScan, callback, queue_size=1)
    plt.show(block=True)  # Begin matplotlib event loop

if __name__ == '__main__':
    listener()
