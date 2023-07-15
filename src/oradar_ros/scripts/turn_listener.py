#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
import math
import numpy as np

class Robot:
    def __init__(self, start_x, start_y, end_x, end_y):
        self.x = start_x
        self.y = start_y
        self.target = np.array([end_x, end_y])
        self.robot_radius = 0.3

    def move_towards(self, direction, distance):
        self.x += distance * math.cos(direction)
        self.y += distance * math.sin(direction)
        print("Moving to position: x={}, y={}".format(self.x, self.y))

    def callback(self, data):
        max_distance = -math.inf
        target_angle = None

        # Calculate the direction to the target in radians
        direction_to_target = math.atan2(self.target[1] - self.y, self.target[0] - self.x)

        for range_index in range(len(data.ranges)):
            distance_to_obstacle = data.ranges[range_index] - self.robot_radius
            angle = -math.pi + (2 * math.pi * range_index / len(data.ranges))

            # Check if the angle is within a 30-degree range of the direction to the target and the distance is larger than the current max_distance
            if abs(angle - direction_to_target) <= math.radians(15) and distance_to_obstacle > max_distance:
                max_distance = distance_to_obstacle
                target_angle = angle

        if target_angle is not None:
            self.move_towards(target_angle, max_distance)
        else:
            print("No clear path towards target.")

def listener():
    rospy.init_node('lidar_listener', anonymous=True)

    # Starting position is (0, 0), target is (3, 3)
    robot = Robot(0, 0, 3, 3)

    rospy.Subscriber("/MS200/scan", LaserScan, robot.callback)
    rospy.spin()

if __name__ == '__main__':
    listener()

