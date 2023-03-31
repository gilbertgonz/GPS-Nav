#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import NavSatFix

if __name__ == '__main__':
    rospy.init_node('gps_publisher')

    pub = rospy.Publisher('current_coordinate', NavSatFix, queue_size=10)

    rate = rospy.Rate(10) # 10 Hz

    while not rospy.is_shutdown():
        # Create a new NavSatFix message and populate its fields
        msg = NavSatFix()
        msg.latitude = 25.714958593757363
        msg.longitude = -80.36150764653273

        # Publish the message
        pub.publish(msg)

        rate.sleep()
