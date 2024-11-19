#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np

class LidarProcessor:
    def __init__(self):
        rospy.init_node('lidar_processor')
        
        # Publishers and Subscribers
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        
        # Parameters
        self.min_distance = 0.3  # Minimum distance for obstacle detection
        self.angular_resolution = 1.0  # degrees
        
    def scan_callback(self, scan_msg):
        # Convert scan data to numpy array
        ranges = np.array(scan_msg.ranges)
        
        # Find minimum distance in front sector (±30 degrees)
        front_indices = len(ranges) // 12  # 30 degrees each side
        front_scan = ranges[:front_indices].tolist() + ranges[-front_indices:].tolist()
        min_distance = min([x for x in front_scan if x > 0.1])  # Filter out invalid readings
        
        # Create Twist message for robot control
        cmd_vel = Twist()
        
        # Simple obstacle avoidance
        if min_distance < self.min_distance:
            # Stop and rotate if obstacle detected
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.5
        else:
            # Move forward if path is clear
            cmd_vel.linear.x = 0.2
            cmd_vel.angular.z = 0.0
            
        self.cmd_vel_pub.publish(cmd_vel)