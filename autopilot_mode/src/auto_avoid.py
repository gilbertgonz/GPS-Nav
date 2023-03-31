#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import numpy as np
from itertools import chain

## TODO: Double FOV sections at 30deg (or maybe even 15deg) instead of 60deg a piece, each section will have different turn intensity

class ObstacleAvoid:
    def __init__(self):
        self.auto_control_sub = rospy.Subscriber('/auto_controller', Bool, self.obstacle_avoid)
        self.sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.mode = False

    def obstacle_avoid(self, data):
        self.mode = data.data
        if self.mode:
            # Initializing Twist and Distance Thresholds
            cmd_vel = Twist()
            dist_thre = 1.0
            emerg_dist_thre = 0.2

            # Filtering 'inf' values from /scan topic
            for i in range(self.range_len):  
                if self.ranges[i] == float("inf"):
                    self.ranges[i] = self.range_max
            ranges = tuple(self.ranges)

            # Calculating relevant mean distance values at 180deg FOV [Left (270deg - 330deg), Middle (330deg - 30deg), Right (30deg - 60deg)]
            range_middle_avg = (sum(ranges[330:359]) + sum(ranges[0:30])) / 60.0
            range_right_avg = sum(ranges[270:330])/ 60.0
            range_left_avg = sum(ranges[30:90])/ 60.0

            # Obstacle Avoidance
            if range_left_avg < dist_thre:  # Avoiding obstacle on left side, turning right
                cmd_vel.linear.x = 0.1
                cmd_vel.angular.z = -0.3
                # rospy.loginfo("Turn right")
                self.cmd_vel_pub.publish(cmd_vel)    
            elif range_right_avg < dist_thre:  # Avoiding obstacle on right side, turning left
                cmd_vel.linear.x = 0.1
                cmd_vel.angular.z = 0.3
                # rospy.loginfo("Turn left")
                self.cmd_vel_pub.publish(cmd_vel)
            elif range_middle_avg < dist_thre:  # Approaching obstacle in the middle, deciding whether to turn right or left
                if range_left_avg < range_right_avg:
                    cmd_vel.linear.x = 0.0
                    cmd_vel.angular.z = -0.3
                    # rospy.loginfo("Turn right")
                    self.cmd_vel_pub.publish(cmd_vel)  
                elif range_left_avg > range_right_avg:
                    cmd_vel.linear.x = 0.0
                    cmd_vel.angular.z = 0.3
                    # rospy.loginfo("Turn left")
                    self.cmd_vel_pub.publish(cmd_vel)
                elif abs(range_left_avg - range_right_avg) < 0.5: 
                    cmd_vel.linear.x = 0.0
                    cmd_vel.angular.z = 0.3
                    # rospy.loginfo("Turn left")
                    self.cmd_vel_pub.publish(cmd_vel)
            for i in chain(range(300, 359), range(0, 60)):  # Emergency stop and reverse if obstacles are too close
                if ranges[i] < emerg_dist_thre:
                    cmd_vel.linear.x = -0.1
                    cmd_vel.angular.z = 0.0
                    # rospy.loginfo("Reverse")
                    self.cmd_vel_pub.publish(cmd_vel)
                
    def laser_callback(self, msg):
        self.range_len= len(msg.ranges) 
        self.range_max = msg.range_max
        self.ranges = list(msg.ranges)

if __name__ == '__main__':
    rospy.init_node('obstacle_avoid')
    obstacle_avoid = ObstacleAvoid()
    rospy.spin()