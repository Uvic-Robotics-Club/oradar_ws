#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int16
import matplotlib.pyplot as plt
import numpy as np
import math
import time
import queue

# Set up plot
fig, ax = plt.subplots()

plot_queue = queue.Queue()

ROBOT_WIDTH = .05
THRESHOLD = .2
LIDAR_FOV = 360
LIDAR_RESOLUTION = .8
LIDAR_MAX_RANGE = 12  # meters
LiDAR_CHANGE_RANGE = 0.5 # meters

# Flag to enable or disable plotting
PLOT_ENABLED_DISTANCE = False
PLOT_ENABLED_SCORE = False
PLOT_ENABLED_FREE_SPACE = False #True # Flag for plotting free space
PLOT_ENABLED_MIDPOINT = False
PLOT_ENABLED_DATA_RANGES = False


# Position of the robot.
robot_position = [0, 0]

# Position of the goal
goal_position = [0, 3]

# Calculate the number of Lidar indices that represent the robot's width
ROBOT_WIDTH_INDICES = int((ROBOT_WIDTH / (2 * np.pi * LIDAR_MAX_RANGE)) * LIDAR_FOV / LIDAR_RESOLUTION)

# Add publisher for "/motor_cmd"
pub = rospy.Publisher('/motor_cmd', Int16, queue_size=1)

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
    

#threshold_free_space starts at index n/2 and ends at index n/2 -1 this way directly in front of the rover does not possess a discontinuity because my theory is discontinuities would occur between 455 and 0

def threshold_free_space(distances, threshold):
    # Initialize free space list
    free_space = [0]*len(distances)
    n = len(distances)
    start = n // 2
    if n > 450:
        for i in range(n):
            # Change from blocked to free space
            index = (start + i) % n
            if distances[index] > threshold:
                free_space[index] = threshold
            # Change from free to blocked space
            elif distances[index] <= threshold:
                free_space[index] = 0
    
    return free_space

#Threshold_groups starts index at n/2 so that there is no discontinuity at 0
def threshold_groups(free_space):
    free_space_groups = []
    start = None
    n = len(free_space)
    begin = n//2
    for i in range(n-1):
        # Starting a new free space
        index = (begin + i) % n
        next_index = (begin + i + 1) % n
        if free_space[index-1] == 0 and free_space[index] != 0:
            start = index
        # Ending a free space
        elif free_space[index] != 0 and free_space[next_index] == 0:
            if start is not None:
                new_groups = (start, index)
                free_space_groups.append(new_groups)
                #print(new_groups)
                start = None
    return free_space_groups

#Free_space_data_points details the angle indices and distance for each pair of points    
def free_space_data_points(free_space, groups):
    free_space_data = []
    for start, end in groups:
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
        free_space_data.append({"distance": distance, "angle_indices": (startrad, endrad)})
  #  for fs in free_space_data:
       # print("Distance:", fs["distance"], "Angle indices:", fs["angle_indices"])
    return free_space_data   

#get_possible_directions outputs midway points in free_space groups
def get_possible_directions(free_space_data, rover_width):


    possible_directions = []

    for fs in free_space_data:
        distance = fs['distance']
        if distance >= rover_width:
            angle_indices = fs['angle_indices']
            #print("Angle Indices: ", angle_indices)  # print the angle_indices

            # ensure angle_indices[0] is less than angle_indices[1]
            a1, a2 = min(angle_indices), max(angle_indices)

            if a2 - a1 > math.pi:
                a1 += 2 * math.pi

            midpoint = (a1 + a2) / 2 % (2 * math.pi)
            
            #print("Midpoint: ", midpoint)  # print the midpoint
            possible_directions.append(midpoint)

    return possible_directions

#find_best_direction finds the direction closest to 
def find_best_direction(midpoints, goal_position, robot_position):

    if not midpoints:
        return None

    goal_direction_dx = goal_position[0] - robot_position[0]
    goal_direction_dy = goal_position[1] - robot_position[1]
    goal_direction = math.atan2(goal_direction_dy, goal_direction_dx)

    differences = [abs(midpoint - goal_direction) for midpoint in midpoints]
    best_index = differences.index(min(differences))
    return midpoints[best_index]


def plot_data(data):
    global ax
    plt.cla()  
    ax.bar(range(len(data.ranges)), data.ranges)
    plt.draw()
    plt.pause(0.001)
    
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
    if len(free_space) > 450:
        plt.cla()
        ax = plt.subplot(1, 1, 1, polar=True)  
        theta = np.linspace(0, 2*np.pi, len(free_space))  # Create an array of equally spaced angle values between 0 and 2*pi
        ax.plot(theta, free_space)  # Plot the free_space data at each angle
        ax.set_theta_zero_location("N")  # Set 0 degrees to the top of the plot
        ax.set_theta_direction(-1)  # Make angles increase in a clockwise direction
        plt.draw()
        plt.pause(0.001)
'''
def plot_polar_midpoints(midpoints_radians, best_direction):
    data = [1]*len(midpoints_radians)  # All angles have same priority

    plt.cla()
    ax = plt.subplot(1, 1, 1, polar=True)  # Create a new subplot in polar format

    for i in range(len(midpoints_radians)):
        color = 'red' if midpoints_radians[i] == best_direction else 'blue'
        ax.plot(midpoints_radians[i], data[i], marker='o', color=color)  # Plot the midpoints at each angle

    ax.set_theta_zero_location("N")  # Set 0 degrees to the top of the plot
    ax.set_theta_direction(-1)  # Make angles increase in a clockwise direction
    plt.draw()
    print("best direction:", best_direction)
    plt.pause(0.001)
'''


def update_plot():
    global ax, plot_queue
    plt.cla()
    data = plot_queue.get()
    midpoints_radians, best_direction = data["midpoints_radians"], data["best_direction"]

    # If best_direction is None, do not update the plot
    if best_direction is None:
        return

    data = [1]*len(midpoints_radians)  # All angles have the same priority

    ax = plt.subplot(1, 1, 1, polar=True)  # Create a new subplot in polar format

    for i in range(len(midpoints_radians)):
        color = 'red' if midpoints_radians[i] == best_direction else 'blue'
        ax.plot(midpoints_radians[i], data[i], marker='o', color=color)  # Plot the midpoints at each angle

    ax.set_theta_zero_location("N")  # Set 0 degrees to the top of the plot
    ax.set_theta_direction(-1)  # Make angles increase in a clockwise direction
    plt.draw()
    print("best direction:", best_direction)


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
    free_space_groups = threshold_groups(free_space)
    free_space_data = free_space_data_points(free_space, free_space_groups)
    possible_directions = get_possible_directions(free_space_data, ROBOT_WIDTH)    
    best_direction = find_best_direction(possible_directions, goal_position, robot_position)
    
    plot_queue.put({"midpoints_radians": possible_directions, "best_direction": best_direction})
    
    turn_mode = True
    #rotation code
    #rotation code
    if best_direction is not None:
        #print("Best Direction in radians " + str(best_direction))
        #rotation_direction = 1 if best_direction > len(finite_ranges) // 2 else 2
        rotation_direction = 1 if best_direction > 3.1415 else 2 #rads
        #if best_direction < 5 or best_direction > (len(finite_ranges) - 5):
        if best_direction < .15 or (best_direction > 6.15): #rads
            cmd_msg = 5
            time.sleep(.1)
            turn_mode = False
        else:
            cmd_msg = rotation_direction
            
    if best_direction is not None:
        pub.publish(cmd_msg)
        #print("Command Msg " + str(cmd_msg))
    
    #done turning and ready to move forward
    if turn_mode == False:
        cur_time = rospy.get_time()
        while rospy.get_time() - cur_time <=.1:
            pub.publish(3)
        pub.publish(5)
        #pause 1 second after moving forward
        time.sleep(1)
    

    
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
    	if possible_directions and best_direction is not None:
    	    plot_polar_midpoints(possible_directions, best_direction)
    	callback.last_update_time = current_time
    	

def listener():
    rospy.init_node('lidar_plot', anonymous=True)
    callback.last_update_time = rospy.get_time()
    rospy.Subscriber("/MS200/scan", LaserScan, callback, queue_size=1)

    run_flag = True
    
    # Call the update_plot function continuously until Ctrl+C is pressed
    while not rospy.is_shutdown() and run_flag:
        update_plot()
        plt.pause(0.001)

    # After Ctrl+C is pressed, stop the plot and exit gracefully
    plt.close()
    rospy.signal_shutdown("Ctrl+C interrupted")
    
if __name__ == '__main__':
    try:
        listener()
    except KeyboardInterrupt:
        pass
