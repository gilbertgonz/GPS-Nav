#!/usr/bin/env python3

# Import the necessary libraries
import rospy # Python library for ROS
import cv2
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
from std_msgs.msg import Int32

imgcheck = 0
img_counter = 1
def callback(data):
  global imgcheck
  imgcheck = data.data
  # print(str(imgcheck))

def callback1(data):
  global imgcheck, img_counter
  br = CvBridge() # Used to convert between ROS and OpenCV images
  img = br.imgmsg_to_cv2(data)
  # print("check")

  if imgcheck > 0:
    cv2.imwrite('img_capture' + str(img_counter) + '.jpg' , img)  # taking screenshot
    img_counter += 1
    print("image saved")
    imgcheck = 0

  # print("counter = " + str(img_counter))

def receive_message():
  rospy.init_node('image_capture')
  rospy.Subscriber('img_capture', Int32, callback)
  rospy.Subscriber('YOLO_img', Image, callback1) # subscribe to the YOLO image with bounding box showing
  rospy.spin()

if __name__ == '__main__':
  receive_message()