#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool, String
from sensor_msgs.msg import LaserScan

## TODO: Get GPS data in simulation

class AutoDecisionNode:
    def __init__(self):
        rospy.init_node('auto_decision_node')

        self.status_string_pub = rospy.Publisher('status_string', String, queue_size=10) 
        self.obstacle_pub = rospy.Publisher('/obstacle_mode', Bool, queue_size=10)
        self.gps_pub = rospy.Publisher('/gps_mode', Bool, queue_size=10)
        self.distance_threshold = rospy.get_param('~distance_threshold', 1.0)
        self.obstacle_mode = False
        self.gps_mode = False
        self.mode = False

        rospy.Subscriber("autopilot_check", Bool, self.switch_mode)
        rospy.Subscriber('/scan', LaserScan, self.lidar_callback)

    # Function for logging switch
    def switch_mode(self, data):
        self.mode = data.data
        if self.mode:
            string = "Switching to Auto"
            self.status_string_pub.publish(string)
            rospy.loginfo(string)
        else:
            self.mode = False # Switching to remote control mode

    def lidar_callback(self, data):
        ranges = list(data.ranges)
        number_of_readings = len(ranges)

        for i in range(len(ranges)):
            if ranges[i] == float("inf"):
                ranges[i] = 12.0
        ranges = tuple(ranges)
        
        range_left1_avg = sum(ranges[330:359]) / 30.0
        range_left2_avg = sum(ranges[300:330]) / 30.0
        range_left3_avg = sum(ranges[270:300]) / 30.0

        range_right1_avg = sum(ranges[0:30]) / 30.0
        range_right2_avg = sum(ranges[30:60]) / 30.0
        range_right3_avg = sum(ranges[60:90]) / 30.0
        range_avgs = [range_left1_avg, range_left2_avg, range_left3_avg, range_right1_avg, range_right2_avg, range_right3_avg]
        
        if self.mode:
            # for i in range(len(range_avgs)):
            #     if (range_avgs[i] < self.distance_threshold):
            if any(x < self.distance_threshold for x in range_avgs):
                self.obstacle_pub.publish(True)
                self.gps_pub.publish(False)

                string = "Objects Close"
                self.status_string_pub.publish(string)
                rospy.loginfo(string)
            else:
                self.obstacle_pub.publish(False)
                self.gps_pub.publish(True)
                    
                string = "Objects Far"
                self.status_string_pub.publish(string)
                # rospy.loginfo(string)
        else:
            self.obstacle_pub.publish(False)
            self.gps_pub.publish(False)
            
            string = "RC Mode"
            self.status_string_pub.publish(string)
            rospy.loginfo(string)

    def receive_message(self):
        rospy.spin()

if __name__ == '__main__':
    auto_decision_node = AutoDecisionNode()
    auto_decision_node.receive_message()