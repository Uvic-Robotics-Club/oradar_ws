#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int16
import matplotlib.pyplot as plt
import numpy as np
import math
import time
from scipy.signal import savgol_filter

# Set up plot
fig, ax = plt.subplots()



ROBOT_WIDTH = .05
THRESHOLD = .2
LIDAR_FOV = 360
LIDAR_RESOLUTION = .8
LIDAR_MAX_RANGE = 12  # meters
LiDAR_CHANGE_RANGE = 0.5 # meters

# Flag to enable or disable plotting
PLOT_ENABLED_DISTANCE = False
PLOT_ENABLED_SCORE = False
PLOT_ENABLED_FREE_SPACE = True #True # Flag for plotting free space
PLOT_ENABLED_MIDPOINT = False
PLOT_ENABLED_DATA_RANGES = False


# Position of the robot.
robot_position = [0, 0]

# Position of the goal
goal_position = [3, 3]

# Calculate the number of Lidar indices that represent the robot's width
ROBOT_WIDTH_INDICES = int((ROBOT_WIDTH / (2 * np.pi * LIDAR_MAX_RANGE)) * LIDAR_FOV / LIDAR_RESOLUTION)

# Add publisher for "/motor_cmd"
pub = rospy.Publisher('/motor_cmd', Int16, queue_size=1)
'''
def calculate_distance_changes(distances):
    changes = []
    for i in range(len(distances) - 1):
        if distances[i] < LiDAR_CHANGE_RANGE:
            changes.append(distances[i] - distances[i+1])
        else:
            changes.append(0)
    return changes
'''
'''
def calculate_distance_changes(distances):
    def moving_average(data, window_size):
        ret = np.cumsum(data, dtype=float)
        ret[window_size:] = ret[window_size:] - ret[:-window_size]
        return ret[window_size - 1:] / window_size

    smoothed_distances = moving_average(distances, 20)  # Using window size of 3
    changes = []
    for i in range(len(smoothed_distances) - 1):
        if smoothed_distances[i] < LiDAR_CHANGE_RANGE:
            changes.append(smoothed_distances[i] - smoothed_distances[i+1])
        else:
            changes.append(0)
    return changes
'''

def remove_nan_and_inf(data):
    # Make a copy of the data to not modify the original
    data_copy = np.array(data)
    
    # Get indices of finite values
    finite_idxs = np.where(np.isfinite(data_copy))[0]  # Extract indices from tuple
    
    # Check if we have at least one finite value
    if len(finite_idxs) == 0:
        # If not, just return a copy filled with zeros (or another value of your choice)
        return np.zeros_like(data_copy)

    # Interpolate to fill NaN values
    data_copy = np.interp(np.arange(len(data_copy)), finite_idxs, data_copy[finite_idxs])
    
    return data_copy
    
'''
def calculate_distance_changes(distances):
    # Apply the Savitzky-Golay filter to smooth distances
    smoothed_distances = savgol_filter(distances, 5, 3)  # Window size: 5, polynomial order: 3

    changes = []
    for i in range(len(smoothed_distances) - 1):
        if smoothed_distances[i] < LiDAR_CHANGE_RANGE:
            changes.append(smoothed_distances[i] - smoothed_distances[i+1])
        else:
            changes.append(0)
    return changes
'''

'''
def threshold_free_space(distances, threshold):
    # Initialize free space list
    free_space = [0]*len(distances)
    for i in range(0, len(distances)-1):
        # Change from blocked to free space
        if distances[i] > threshold:
            free_space[i] = threshold;
            # free_space[i] = distances[i] plots distance
        # Change from free to blocked space
        elif distances[i] <= threshold:
            free_space[i] = 0
    print(distances[1])
    return free_space
'''
#threshold_free_space starts at index n/2 and ends at index n/2 -1 this way directly in front of the rover does not possess a discontinuity
def threshold_free_space(distances, threshold):
    # Initialize free space list
    free_space = [0]*len(distances)
    
    n = len(distances)
    start = n // 2
    
    for i in range(n):
        # Change from blocked to free space
        index = (start + i) % n
        if distances[index] > threshold:
            free_space[index] = threshold
        # Change from free to blocked space
        elif distances[index] <= threshold:
            free_space[index] = 0
    
    return free_space

def identify_free_space(distances, threshold):
    # Initialize free space list
    free_space = [0]*len(distances)
    for i in range(0, len(distances)-1):
        # Change from blocked to free space
        if distances[i] > threshold:
            free_space[i] = threshold
        # Change from free to blocked space
        elif distances[i] <= threshold:
            free_space[i] = 0

    # Identifying free spaces that are bounded by zeros on both sides
    
    free_spaces = []
    start = None
    for i in range(1, len(free_space)-1):
        # Starting a new free space
        if free_space[i-1] == 0 and free_space[i] != 0:
            start = i
        # Ending a free space
        elif free_space[i] != 0 and free_space[i+1] == 0:
            if start is not None:
                free_spaces.append((start, i))
                
                start = None
                
    # Calculate the physical distances for each free space
    free_space_data = []
    for start, end in free_spaces:
        #print("Start of Group " + str(start) + " End of Group " + str(end))
        a = free_space[start]
        b = free_space[end]
        #print("a is " +str(a) + "threshold" + str(threshold) + "b is " +str(b))
        startrad = start*2*np.pi/len(free_space)
        endrad = end*2*np.pi/len(free_space)
        
        
        theta = abs(endrad - startrad)  # Assuming end and start are in degrees. Convert to radians if not.
        
        # Distance formula
        distance = math.sqrt(a**2 + b**2 - 2*a*b*math.cos(theta))  # Use math.radians if theta is not in radians
        #print("Distance is " + str(distance))
        free_space_data.append({"distance": distance, "angle_indices": (start, end)})
        
    # Print the data
    #for fs in free_space_data:
        #print("Distance:", fs["distance"], "Angle indices:", fs["angle_indices"])
    
    return free_space_data


def get_possible_directions(distances, threshold, rover_width):
    free_spaces = identify_free_space(distances, threshold)
    possible_directions = []

    for fs in free_spaces:
        distance = fs['distance']
        if distance >= rover_width:
            angle_indices = fs['angle_indices']
            midpoint = ((angle_indices[0] + angle_indices[1])%len(distances)) / 2
            possible_directions.append(midpoint)
            #print(midpoint)

    return possible_directions


def plot_data(data):
    global ax
    plt.cla()  
    ax.bar(range(len(data.ranges)), data.ranges)
    plt.draw()
    plt.pause(0.001)
    
'''
def plot_free_space(free_space):
    plt.cla()
    ax = plt.subplot(1, 1, 1, polar=True)  
    theta = np.linspace(0, 2*np.pi, len(free_space))  # Create an array of equally spaced angle values between 0 and 2*pi
    ax.plot(theta, free_space)  # Plot the free_space data at each angle
    ax.set_theta_zero_location("N")  # Set 0 degrees to the top of the plot
    ax.set_theta_direction(-1)  # Make angles increase in a clockwise direction
    plt.draw()
    plt.pause(0.001)
'''

'''
def plot_smoothed_distances(smoothed_distances):
    global ax
    plt.cla()
    angles = np.linspace(0, LIDAR_FOV, len(smoothed_distances))
    ax.plot(angles, smoothed_distances)
    ax.set_xlabel('Angle (degrees)')
    ax.set_ylabel('Smoothed Distance')
    plt.draw()
    plt.pause(0.001)

'''
def plot_data_ranges(data_ranges):
    plt.cla()
    ax = plt.subplot(1, 1, 1, polar=True)  
    theta = np.linspace(0, 2*np.pi, len(data_ranges))  # Create an array of equally spaced angle values between 0 and 2*pi
    ax.plot(theta, data_ranges)  # Plot the smoothed_distances at each angle
    ax.set_theta_zero_location("N")  # Set 0 degrees to the top of the plot
    ax.set_theta_direction(-1)  # Make angles increase in a clockwise direction
    plt.title("Data Ranges")
    plt.draw()
    plt.pause(0.001)



def plot_free_space_polar(free_space):
    plt.cla()
    ax = plt.subplot(1, 1, 1, polar=True)  
    theta = np.linspace(0, 2*np.pi, len(free_space))  # Create an array of equally spaced angle values between 0 and 2*pi
    ax.plot(theta, free_space)  # Plot the free_space data at each angle
    ax.set_theta_zero_location("N")  # Set 0 degrees to the top of the plot
    ax.set_theta_direction(-1)  # Make angles increase in a clockwise direction
    plt.draw()
    plt.pause(0.001)

'''
def plot_free_space(free_space_data):
    plt.cla()
    ax = plt.subplot(1, 1, 1, polar=True)  # Create a new subplot in polar format
    for space in free_space_data:
        start, end = space["angle_indices"]
        theta = np.linspace(math.radians(start), math.radians(end), end-start+1)  # Convert FOV angles to radians
        distance = space["distance"]
        ax.plot(theta, [distance]*(end-start+1))  # Plot the distances at each angle
    ax.set_theta_zero_location("N")  # Set 0 degrees to the top of the plot
    ax.set_theta_direction(-1)  # Make angles increase in a clockwise direction
    plt.draw()
    plt.pause(0.001)
'''
  
def plot_polar_midpoints(midpoints):
    # Convert angles to radians and normalize
    midpoints_radians = [np.radians(midpoint) for midpoint in midpoints]
    data = [1]*len(midpoints_radians)  # All angles have same priority
    
    plt.cla() 
    ax = plt.subplot(1, 1, 1, polar=True)  # Create a new subplot in polar format
    ax.plot(midpoints_radians, data, marker='o')  # Plot the midpoints at each angle
    ax.set_theta_zero_location("N")  # Set 0 degrees to the top of the plot
    ax.set_theta_direction(-1)  # Make angles increase in a clockwise direction
    plt.draw()
    plt.pause(0.001)


def plot_scores(scores, best_direction):
    global ax
    plt.cla()
    angles = np.linspace(0, LIDAR_FOV, len(scores))
    ax.plot(angles, scores)
    ax.scatter([angles[best_direction]], [scores[best_direction]], color = 'r')
    ax.set_xlabel('Angle (degrees)')
    ax.set_ylabel('Score')
    plt.draw()
    plt.pause(0.001)


def callback(data):
    global PLOT_ENABLED_DISTANCE, PLOT_ENABLED_SCORE, PLOT_ENABLED_FREE_SPACE, robot_position, goal_position
    current_time = rospy.get_time()
    finite_ranges = remove_nan_and_inf(data.ranges)
  #  smoothed_distances = savgol_filter(finite_ranges, 5, 3)  
    free_space = threshold_free_space(data.ranges, THRESHOLD)
    possible_directions = get_possible_directions(data.ranges, THRESHOLD, ROBOT_WIDTH)    
    
    if PLOT_ENABLED_DISTANCE and current_time - callback.last_update_time >= 1.0:     
        #plot_free_space_polar(finite_ranges)
        callback.last_update_time = current_time
     
    if PLOT_ENABLED_DATA_RANGES and current_time - callback.last_update_time >= 1.0:
    	#plot_data_ranges(finite_ranges) # subbed in 'data.ranges' for 'smoothed_distances'
    	callback.last_update_time = current_time
    	
    if PLOT_ENABLED_FREE_SPACE and current_time - callback.last_update_time >= 1.0:
        #free_space = identify_free_space(finite_ranges, THRESHOLD)
        plot_free_space_polar(free_space)
        callback.last_update_time = current_time
    
    
    if PLOT_ENABLED_MIDPOINT and current_time - callback.last_update_time >= 1.0:
    	#plot_polar_midpoints(possible_directions)
    	callback.last_update_time = current_time
    	
    
    goal_direction_dx = goal_position[0] - robot_position[0]
    goal_direction_dy = goal_position[1] - robot_position[1]
    goal_direction = math.atan2(goal_direction_dy, goal_direction_dx)
'''
    scores = []
    for i, distance in enumerate(free_space):
        if distance > ROBOT_WIDTH:
            angle_difference = abs(i - len(free_space) // 2 - goal_direction)
            score = distance / (1 + angle_difference)
            scores.append(score)
        else:
            scores.append(0)

    best_direction = scores.index(max(scores))
    rotation_direction = 1 if best_direction > len(free_space) // 2 else 2

    if best_direction < 10 or best_direction > 350:
        cmd_msg = 5
        time.sleep(1)
    else:
        cmd_msg = rotation_direction

    pub.publish(cmd_msg)
    if cmd_msg == 5:
        cur_time = rospy.get_time()
        while rospy.get_time() - cur_time <=.1:
            pub.publish(3)
        pub.publish(5)
        time.sleep(1)

    if PLOT_ENABLED_SCORE and current_time - callback.last_update_time >= 1.0:
        plot_scores(scores, best_direction)
        callback.last_update_time = current_time

    step_size = 0.1
    robot_position[0] += step_size * math.cos(best_direction)
    robot_position[1] += step_size * math.sin(best_direction)
'''
def listener():
    rospy.init_node('lidar_plot', anonymous=True)
    callback.last_update_time = rospy.get_time()
    rospy.Subscriber("/MS200/scan", LaserScan, callback, queue_size=1)
    plt.show(block=True)

if __name__ == '__main__':
    listener()

