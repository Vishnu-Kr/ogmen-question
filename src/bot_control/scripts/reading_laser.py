#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan

def scan_callback(scan_msg):
    filtered_ranges = scan_msg.ranges[0:120] # take only the first 120 degrees
    filtered_scan = LaserScan()
    filtered_scan.header = scan_msg.header
    filtered_scan.angle_min = scan_msg.angle_min
    filtered_scan.angle_max = scan_msg.angle_min + 1.0472 # 60 degrees in radians
    filtered_scan.angle_increment = scan_msg.angle_increment
    filtered_scan.time_increment = scan_msg.time_increment
    filtered_scan.scan_time = scan_msg.scan_time
    filtered_scan.range_min = scan_msg.range_min
    filtered_scan.range_max = scan_msg.range_max
    filtered_scan.ranges = filtered_ranges
    filtered_scan.intensities = scan_msg.intensities
    filtered_scan_pub.publish(filtered_scan)

if __name__ == '__main__':
    rospy.init_node('filtered_scan_node')
    scan_sub = rospy.Subscriber('/scan', LaserScan, scan_callback)
    filtered_scan_pub = rospy.Publisher('/filtered_scan', LaserScan, queue_size=10)
    rospy.spin()
