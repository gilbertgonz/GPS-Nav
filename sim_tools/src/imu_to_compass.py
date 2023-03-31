#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
import math

class ImuToCompass:
    def __init__(self):
        rospy.init_node('imu_to_compass_node', anonymous=True)

        # Subscribers
        self.imu_sub = rospy.Subscriber('/imu', Imu, self.imu_callback, queue_size=10)

        # Publishers
        self.compass_pub = rospy.Publisher('/compass', Float32, queue_size=10)

    def imu_callback(self, imu_data):
        # Extract yaw angle from the IMU data
        q = imu_data.orientation
        yaw = 180.0 / 3.14159265 * math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        # Normalize yaw angle to [0, 360] degrees
        yaw = yaw % 360

        # Convert yaw angle to compass angle with clockwise rotation
        compass_yaw = (360 - yaw) % 360

        # Publish compass angle
        self.compass_pub.publish(compass_yaw)


if __name__ == '__main__':
    try:
        imu_to_compass = ImuToCompass()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
