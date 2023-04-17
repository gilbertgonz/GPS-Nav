#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

def cmd_vel_callback(msg):
    if msg.linear.x > 0:
        pub.publish(True)

if __name__ == '__main__':
    rospy.init_node('metal_detector_sim')

    pub = rospy.Publisher('metal_detection', Bool, queue_size=10)

    sub = rospy.Subscriber('cmd_vel', Twist, cmd_vel_callback)

    rospy.spin()
