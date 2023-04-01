#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import numpy as np
from itertools import chain

class ObstacleAvoid:
    def __init__(self):
        self.auto_control_sub = rospy.Subscriber('/obstacle_mode', Bool, self.obstacle_avoid, queue_size=10)
        self.sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.mode = False

    def obstacle_avoid(self, data):
        self.mode = data.data
        if self.mode:
            # Initializing Twist and Distance Thresholds
            cmd_vel = Twist()
            dist_thre = 1.0
            emerg_dist_thre = 0.25
            max_speed = 0.2

            # Filtering 'inf' values from /scan topic
            for i in range(self.range_len):  
                if self.ranges[i] == float("inf"):
                    self.ranges[i] = self.range_max
            ranges = tuple(self.ranges)

            # Calculating relevant mean distance values of 6 equally divided sections (3 on the right and 3 on the left) at 180deg FOV
            range_left1_avg = sum(ranges[330:359]) / 30.0
            range_left2_avg = sum(ranges[300:330]) / 30.0
            range_left3_avg = sum(ranges[270:300]) / 30.0

            range_right1_avg = sum(ranges[0:30]) / 30.0
            range_right2_avg = sum(ranges[30:60]) / 30.0
            range_right3_avg = sum(ranges[60:90]) / 30.0

            right_avg_total = (range_right1_avg + range_right2_avg + range_right3_avg) / 3.0
            left_avg_total = (range_left1_avg + range_left2_avg + range_left3_avg) / 3.0

            # Implementing Obstacle Avoidance (which is a modified variation of the Braitenburg vehicle algorithm)
            ## Avoiding obstacle on left side, turning right
            if range_left1_avg < dist_thre:  
                cmd_vel.linear.x = max_speed / 2
                cmd_vel.angular.z = 0.7
                # rospy.loginfo("Turn left 1")
                self.cmd_vel_pub.publish(cmd_vel)
            if range_left2_avg < dist_thre:
                cmd_vel.linear.x = max_speed / 2
                cmd_vel.angular.z = 0.6
                # rospy.loginfo("Turn left 2")
                self.cmd_vel_pub.publish(cmd_vel) 
            if range_left3_avg < dist_thre:
                cmd_vel.linear.x = max_speed
                cmd_vel.angular.z = 0.5
                # rospy.loginfo("Turn left 3")
                self.cmd_vel_pub.publish(cmd_vel)  

            ## Avoiding obstacle on right side, turning left
            if range_right1_avg < dist_thre:  
                cmd_vel.linear.x = max_speed / 2
                cmd_vel.angular.z = -0.7
                # rospy.loginfo("Turn right 1")
                self.cmd_vel_pub.publish(cmd_vel)
            if range_right2_avg < dist_thre:
                cmd_vel.linear.x = max_speed / 2
                cmd_vel.angular.z = -0.6
                # rospy.loginfo("Turn right 2")
                self.cmd_vel_pub.publish(cmd_vel) 
            if range_right3_avg < dist_thre:
                cmd_vel.linear.x = max_speed
                cmd_vel.angular.z = -0.5
                # rospy.loginfo("Turn right 3")
                self.cmd_vel_pub.publish(cmd_vel)   

            ## Emergency stop and reverse if obstacles are too close
            for i in chain(range(300, 359), range(0, 60)):  
                if ranges[i] < emerg_dist_thre:
                    cmd_vel.linear.x = -max_speed
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