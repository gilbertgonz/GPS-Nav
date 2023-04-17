#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import NavSatFix
import time

if __name__ == '__main__':
    rospy.init_node('gps_publisher')

    pub = rospy.Publisher('current_coordinate', NavSatFix, queue_size=10)

    rate = rospy.Rate(1) # 1 Hz

    waypoints = [(25.714958593757363, -80.36150764653273), (25.714958485217363, -80.36239754653273), (25.714658433757363, -80.36473905153273)]
    current_waypoint_index = 0

    while not rospy.is_shutdown():
        # Create a new NavSatFix message and populate its fields with the current waypoint
        msg = NavSatFix()
        msg.latitude = waypoints[current_waypoint_index][0]
        msg.longitude = waypoints[current_waypoint_index][1]

        # Publish the message
        pub.publish(msg)

        # Move to the next waypoint
        current_waypoint_index = (current_waypoint_index + 1) % len(waypoints)

        # Wait one second before publishing the next message
        time.sleep(1)

        rate.sleep()
