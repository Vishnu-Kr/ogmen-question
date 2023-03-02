#!/usr/bin/env python3

import rospy
import math
import tf
from geometry_msgs.msg import Twist, PoseStamped, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import paramiko

waypoints = []
waypoint_index = 0
current_position = None
current_yaw = None

def read_waypoints():
    global waypoints
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect('ssh_server_address', username='username', password='password')
    sftp = ssh.open_sftp()
    sftp.get('/path/to/waypoints.txt', 'waypoints.txt')
    with open('waypoints.txt', 'r') as f:
        for line in f:
            coords = line.strip().split(',')
            x = float(coords[0])
            y = float(coords[1])
            yaw = float(coords[2])
            waypoints.append((x, y, yaw))
    sftp.close()
    ssh.close()

def get_odom(msg):
    global current_position, current_yaw
    current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    current_yaw = euler[2]

def distance(p1, p2):
    return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

def move_to_next_waypoint():
    global waypoint_index
    global current_position
    global current_yaw
    waypoint = waypoints[waypoint_index]
    while distance(current_position, waypoint) > 0.1:
        vel_msg = Twist()
        distance_to_waypoint = distance(current_position, waypoint)
        angle_to_waypoint = math.atan2(waypoint[1] - current_position[1], waypoint[0] - current_position[0])
        angle_difference = angle_to_waypoint - current_yaw
        if angle_difference > math.pi:
            angle_difference -= 2*math.pi
        elif angle_difference < -math.pi:
            angle_difference += 2*math.pi
        if abs(angle_difference) > math.pi/4:
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.2 if angle_difference > 0 else -0.2
        else:
            vel_msg.linear.x = min(distance_to_waypoint, 0.5)
            vel_msg.angular.z = 0.5 * angle_difference
        cmd_vel_pub.publish(vel_msg)
        rate.sleep()
    waypoint_index += 1
    if waypoint_index == len(waypoints):
        spin()

def spin():
    global current_yaw
    vel_msg = Twist()
    rate = rospy.Rate(10)
    t0 = rospy.Time.now().to_sec()
    while (rospy.Time.now().to_sec() - t0) < 6*math.pi:
        vel_msg.angular.z = 1.0
        cmd_vel_pub.publish(vel_msg)
        rate.sleep()
    vel_msg.angular.z = 0.0
    cmd_vel_pub.publish(vel_msg)

if __name__ == '__main__':
    rospy.init_node('move_to_waypoints')
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/odom', Odometry, get_odom)
    read_waypoints()
    rate = rospy.Rate(10)
