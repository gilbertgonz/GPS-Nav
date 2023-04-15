#!/usr/bin/env python3

# Import the necessary libraries
import rospy # Python library for ROS
import cv2
from sensor_msgs.msg import Image, NavSatFix # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
from std_msgs.msg import Int32, String
import datetime

imgcheck = 0

def callback(data):
  global imgcheck
  imgcheck = data.data
  # print("imgcheck: " + str(imgcheck))

def callback1(img_data):
  global imgcheck, lat, long
  br = CvBridge() # Used to convert between ROS and OpenCV images
  img = br.imgmsg_to_cv2(img_data)

  if imgcheck > 0:
    try:
      text = f"GPS: {round(lat, 12)}, {round(long, 12)}"
      cv2.putText(img, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2, cv2.LINE_AA)
    except NameError:
      text = "GPS:"
      cv2.putText(img, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2, cv2.LINE_AA)

    now = datetime.datetime.now()
    filename = now.strftime("%Y-%m-%d_%H-%M-%S") + ".jpg"
    cv2.imwrite('../first_ws/src/mine_cam_finder/captured_images/' + str(filename), img)  # taking screenshot
    string = "Image saved"
    status_string_pub.publish(string)
    print(string)
    imgcheck = 0

  # print("counter = " + str(img_counter))

def current_location_callback(gps_data):
  global lat, long
  lat = gps_data.latitude
  long = gps_data.longitude

def receive_message():
  rospy.init_node('image_capture')
  rospy.Subscriber('img_capture', Int32, callback)
  rospy.Subscriber('YOLO_img', Image, callback1) # subscribe to the YOLO image with bounding box showing 'YOLO_img'
  rospy.Subscriber('current_coordinate', NavSatFix, current_location_callback, queue_size=1)
  rospy.spin()

if __name__ == '__main__':
  status_string_pub = rospy.Publisher('status_string', String, queue_size=1)
  receive_message()