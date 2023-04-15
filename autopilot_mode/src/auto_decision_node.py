#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool, String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3

## TODO: Get GPS data in simulation

class AutoDecisionNode:
    def __init__(self):
        rospy.init_node('auto_decision_node')

        self.status_string_pub = rospy.Publisher('status_string', String, queue_size=10) 
        self.obstacle_pub = rospy.Publisher('/obstacle_mode', Bool, queue_size=10)
        self.gps_pub = rospy.Publisher('/gps_mode', Bool, queue_size=10)
        self.distance_threshold = rospy.get_param('~distance_threshold', 0.8)
        self.twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
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
        # print(number_of_readings)  # 1147

        for i in range(len(ranges)):
            if ranges[i] == float("inf"):
                ranges[i] = 12.0
        ranges = tuple(ranges)
        
        range_left1_avg = sum(ranges[number_of_readings*11//12:number_of_readings]) / (number_of_readings//12) # sum(ranges[330:359]) / 30.0
        range_left2_avg = sum(ranges[number_of_readings*5//6:number_of_readings*11//12]) / (number_of_readings//12) # sum(ranges[300:330]) / 30.0
        range_left3_avg = sum(ranges[number_of_readings*3//4:number_of_readings*5//6]) / (number_of_readings//12) # sum(ranges[270:300]) / 30.0

        range_right1_avg = sum(ranges[0:number_of_readings//12]) / (number_of_readings//12) # sum(ranges[0:30]) / 30.0
        range_right2_avg = sum(ranges[number_of_readings//12:number_of_readings//6]) / (number_of_readings//12) # sum(ranges[30:60]) / 30.0
        range_right3_avg = sum(ranges[number_of_readings//6:number_of_readings//4]) / (number_of_readings//12) # sum(ranges[60:90]) / 30.0
        
        range_avgs = [range_left1_avg, range_left2_avg, range_left3_avg, range_right1_avg, range_right2_avg, range_right3_avg]

        # print("middle " + str(ranges[0]))
        # print("rear   " + str(ranges[number_of_readings//2]))
        # print("left   " + str(ranges[number_of_readings//4]))
        # print("right  " + str(ranges[number_of_readings*3//4]) + "\n")

        # print("left  " + str(range_avgs[0:3]))
        # print("right " + str(range_avgs[3:6]) + "\n")
        
        if self.mode:
            if any(x < self.distance_threshold for x in range_avgs):
                self.obstacle_pub.publish(True)
                self.gps_pub.publish(False)

                string = "Objects Close"
                self.status_string_pub.publish(string)
                # rospy.loginfo(string)
            else:
                self.obstacle_pub.publish(False)
                self.gps_pub.publish(True)
                    
                string = "Objects Far"
                self.twist_pub.publish(Twist(linear=Vector3(0, 0, 0), angular=Vector3(0, 0, 0)))
                self.status_string_pub.publish(string)
                rospy.loginfo(string)
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