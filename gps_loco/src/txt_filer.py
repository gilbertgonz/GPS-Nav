#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import NavSatFix
import datetime
import time

timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
# filename = f'/home/gilberto/first_ws/src/gps_loco/src/txt_files/gps_data_{timestamp}.txt'
filename = '/home/gilberto/first_ws/src/gps_loco/src/txt_files/test.txt'

def callback_gps(data):
    if data.latitude and data.longitude:
        with open(filename, "a") as file:
            file.write("{}, {}\n".format(data.latitude, data.longitude))
    else:
        rospy.loginfo("No current coordinates")

    time.sleep(2)


def callback_mine(data):
    if data.latitude and data.longitude:
        with open(filename, "a") as file:
            file.write("{}, {}, mine\n".format(data.latitude, data.longitude))
    else:
        rospy.loginfo("No mine coordinates")

if __name__ == '__main__':
    rospy.init_node('gps_txt_generator', anonymous=True)
    rospy.Subscriber("/current_coordinate", NavSatFix, callback_gps)
    rospy.Subscriber("/mine_coordinate", NavSatFix, callback_mine)
    rospy.spin()
    