#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np
import os
import time

class ImageSubscriber:
    def __init__(self):
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.callback)
        print("recieved")
        self.bridge = CvBridge()
        self.counter = 0

    def callback(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data)

        file_name = 'image_' + str(self.counter) + '.png'

        cv2.imwrite(file_name, cv_image)
        rospy.loginfo('Saved image: ' + file_name)
        
        self.counter += 1

        # Save the image every 1.5 seconds
        time.sleep(1.5)
        

if __name__ == '__main__':
    rospy.init_node('image_taker_node')
    img_sub = ImageSubscriber()
    rospy.spin()
