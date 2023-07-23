#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int16
#import matplotlib.pyplot as plt
#from matplotlib.animation import FuncAnimation
import numpy as np
import math
import time

# Set up plot
#fig, ax = plt.subplots()

ROBOT_WIDTH = .3
THRESHOLD = .6
LIDAR_FOV = 360
LIDAR_RESOLUTION = .8
LIDAR_MAX_RANGE = 12  # meters
LiDAR_CHANGE_RANGE = 0.5 # meters

# Flag to enable or disable plotting
PLOT_ENABLED_DISTANCE = False
PLOT_ENABLED_SCORE = False
PLOT_ENABLED_FREE_SPACE = False #True # Flag for plotting free space
PLOT_ENABLED_MIDPOINT = True
PLOT_ENABLED_DATA_RANGES = False


# Position of the robot.
robot_position = [0, 0]

# Position of the goal
goal_position = [0, 3]

global_angle = 0
first_scan_flag = True

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
                free_space[index] = distances[index]
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
        free_space_data.append({"distance": distance, "angle_indices": (start, end)})
    #for fs in free_space_data:
        #print("Distance:", fs["distance"], "Angle indices:", fs["angle_indices"])
    return free_space_data   

#get_possible_directions outputs midway points in free_space groups
def get_possible_directions(free_space_data, rover_width,free_space):


    possible_directions = []

    for fs in free_space_data:
        distance = fs['distance']
        if distance >= rover_width:
            angle_indices = fs['angle_indices']
            #print("Angle Indices: ", angle_indices)  # print the angle_indices

            # ensure angle_indices[0] is less than angle_indices[1]
            a1, a2 = min(angle_indices), max(angle_indices)

            if a2 - a1 > len(free_space)/2:
                a1 += len(free_space)

            midpoint = (a1 + a2) // 2 % (len(free_space))
            
          #  print("Midpoint: ", midpoint)  # print the midpoint
            possible_directions.append({"midpoint":midpoint,"DistanceToPoint":free_space[midpoint]})

    return possible_directions

#find_best_direction finds the direction closest to 
def find_best_direction(midpoints_data, goal_position, free_space):
    if not midpoints_data:
        return None

    # Weights for the distance and direction components of the score
    # You can adjust these to suit your needs
    distance_weight = 0.4
    direction_weight = 0.6


    scores = []
    for data in midpoints_data:
        midpoint = data['midpoint']
        distance = data['DistanceToPoint']

        # Calculate the direction component of the score
        if midpoint > len(free_space)/2:
            direction_difference =  len(free_space) - midpoint
        else:
            direction_difference = midpoint #may need to change
        direction_score = 1 - direction_difference/len(free_space) # Normalize to range [0, 1]

        # Calculate the overall score
        #score = distance_weight * distance + direction_weight * direction_score
        score = direction_weight * direction_score #+ distance*distance_weight
        scores.append(score)
       # print(data)
      #  print(score)

    # Find the index of the highest score
    best_index = scores.index(max(scores))
    #print("Best index is the following: " + str(best_index))

    return midpoints_data[best_index]

'''

def plot_data(data):
    global ax
    plt.cla()  
    ax.bar(range(len(data.ranges)), data.ranges)
    plt.draw()
    plt.pause(0.001)
    
def plot_data_ranges(data_ranges):
    if len(data_ranges):
        plt.cla()
        ax = plt.subplot(1, 1, 1, polar=True)  
        theta = np.linspace(0, 2*np.pi, len(data_ranges))  # Create an array of equally spaced angle values between 0 and 2*pi
        ax.plot(theta, data_ranges)  # Plot the smoothed_distances at each angle
        ax.set_theta_zero_location("N")  # Set 0 degrees to the top of the plot
        ax.set_theta_direction(-1)  # Make angles increase in a clockwise direction
        plt.title("Data Ranges")
        plt.draw()
        plt.pause(0.001)rrrr


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
    plt.pause(0.001)
'''
'''
def plot_polar_midpoints_and_best(midpoints_data, best_direction_data, total_angle_indices):
    # Scale the angle indices to be in the range [0, 2*pi]
    midpoints_radians = [2 * np.pi * (data['midpoint'] / total_angle_indices) for data in midpoints_data]
    best_direction_radian = 2 * np.pi * (best_direction_data['midpoint'] / total_angle_indices)

    # Create data for plotting
    data = [1]*len(midpoints_radians)  # All midpoints have the same priority
    best_data = 1  # Best direction has the same priority

    plt.cla() 
    ax = plt.subplot(1, 1, 1, polar=True)  # Create a new subplot in polar format

    # Plot the midpoints at each angle
    ax.plot(midpoints_radians, data, 'bo')

    # Plot the best direction in red
    ax.plot(best_direction_radian, best_data, 'ro')

    ax.set_theta_zero_location("N")  # Set 0 degrees to the top of the plot
    ax.set_theta_direction(-1)  # Make angles increase in a clockwise direction

    plt.draw()
    plt.pause(0.001)
'''

def callback(data):
  #  global PLOT_ENABLED_DISTANCE, PLOT_ENABLED_SCORE, PLOT_ENABLED_FREE_SPACE, robot_position, goal_position
    global goal_position
    
    #global global_angle, first_scan_flag
    current_time = rospy.get_time()
    finite_ranges = remove_nan_and_inf(data.ranges)
  #  smoothed_distances = savgol_filter(finite_ranges, 5, 3)  
    free_space = threshold_free_space(data.ranges, THRESHOLD)
    free_space_groups = threshold_groups(free_space)
    free_space_data = free_space_data_points(free_space, free_space_groups)
    possible_directions = get_possible_directions(free_space_data, ROBOT_WIDTH,free_space)    
    best_direction = find_best_direction(possible_directions, goal_position, free_space)
    
    
    	#print(best_direction)

    turn_mode = True
    #rotation code
    #rotation code
    if best_direction is not None:
        best_dir = best_direction['midpoint']
     #   print("Best Direction in radians " + str(best_direction))
        #if first_scan_flag:
        	#global_angle += best_direction%(len(free_space))
        	#print("Global Angle: " , global_angle)
        	#first_scan_flag = False
         
        #rotation_direction = 1 if best_direction > len(finite_ranges) // 2 else 2
        rotation_direction = 1 if best_dir > len(free_space)/2 else 2 #rads
        #if best_direction < 5 or best_direction > (len(finite_ranges) - 5):
        #print(best_dir)
        if best_dir < 15 or (best_dir > (len(free_space)-15)): #rads
            cmd_msg = 5
            time.sleep(.2)
            turn_mode = False
        else:
            cmd_msg = rotation_direction
        
        pub.publish(cmd_msg) 
        time.sleep(.01)  
        pub.publish(5)
    
        
     #   print("Command Msg " + str(cmd_msg))
    
    #done turning and ready to move forward
    if turn_mode == False:
        cur_time = rospy.get_time()
        #while rospy.get_time() - cur_time <=.1:
        pub.publish(3)
        time.sleep(.05)
            #print(rospy.get_time() - cur_time)
        pub.publish(5)
        #first_scan_flag = True
        #pause 1 second after moving forward
        time.sleep(1)
        #print("Pause")
    
    
    if PLOT_ENABLED_DISTANCE and current_time - callback.last_update_time >= 1.0:     
        #plot_free_space_polar(finite_ranges)
        callback.last_update_time = current_time
     
    if PLOT_ENABLED_DATA_RANGES and current_time - callback.last_update_time >= 1.0:
    	#plot_data_ranges(finite_ranges) # subbed in 'data.ranges' for 'smoothed_distances'
    	callback.last_update_time = current_time
    	
    if PLOT_ENABLED_FREE_SPACE and current_time - callback.last_update_time >= 1.0:
        #free_space = identify_free_space(finite_ranges, THRESHOLD)
        #plot_free_space_polar(free_space)
        callback.last_update_time = current_time
    
    
    #if PLOT_ENABLED_MIDPOINT and current_time - callback.last_update_time >= 1.0:
    	#if possible_directions and best_direction is not None:
    	    #plot_polar_midpoints_and_best(possible_directions, best_direction,len(free_space))
    	#callback.last_update_time = current_time
    	
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

    #plt.show(block=True)  # Move this line here
    rospy.spin()

if __name__ == '__main__':
    listener()

