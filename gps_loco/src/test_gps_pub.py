#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import NavSatFix
import time

file_path = "/home/gilberto/first_ws/src/gps_loco/src/txt_files/test3.txt"

def read_file():
    with open(file_path, "r") as file:
        lines = file.readlines()
        for line in lines:
            yield line.strip()

def talker():
    pub = rospy.Publisher('current_coordinate', NavSatFix, queue_size=10)
    rospy.init_node('test_gps_pub', anonymous=True)
    rate = rospy.Rate(2)  # Publish at a rate of X message per second
    msg = NavSatFix()
    
    lines = read_file()
    for line in lines:
        values = line.split(",")
        msg.latitude = float(values[0])
        msg.longitude = float(values[1])
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
