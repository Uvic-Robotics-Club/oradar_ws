#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int16

def callback(data):
    radius_limit = 0.10  # 10 cm, note this is in meters
    for range_index in range(len(data.ranges)):
        if (data.ranges[range_index] < radius_limit and data.ranges[range_index] != 0) :
            print("Data point within 10cm at angle index: {}, distance: {} meters".format(range_index, data.ranges[range_index]))
            # Publish "1" to turn the motor on
            motor_pub.publish(Int16(data=1))
            return  # Exit after finding the first point within the limit
    # If no point is found within the limit, turn the motor off
    motor_pub.publish(Int16(data=0))

def listener():
    global motor_pub  # Make the publisher globally accessible
    rospy.init_node('lidar_motor_controller', anonymous=True)
    motor_pub = rospy.Publisher('/motor_cmd', Int16, queue_size=1)  # Initialize the publisher
    rospy.Subscriber("/MS200/scan", LaserScan, callback)  # replace 'lidar_topic' with the appropriate topic name
    rospy.spin()

if __name__ == '__main__':
    listener()

