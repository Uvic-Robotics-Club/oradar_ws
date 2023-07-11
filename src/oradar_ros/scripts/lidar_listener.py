#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan

def callback(data):
    radius_limit = 0.10  # 10 cm, note this is in meters
    for range_index in range(len(data.ranges)):
        if (data.ranges[range_index] < radius_limit and data.ranges[range_index] != 0) :
            print("Data point within 10cm at angle index: {}, distance: {} meters".format(range_index, data.ranges[range_index]))
def listener():
    rospy.init_node('lidar_listener', anonymous=True)
    rospy.Subscriber("/MS200/scan", LaserScan, callback)  # replace 'lidar_topic' with the appropriate topic name
    rospy.spin()

if __name__ == '__main__':
    listener()

