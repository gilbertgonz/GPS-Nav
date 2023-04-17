#!/usr/bin/env python3

# Import the necessary libraries
import rospy # Python library for ROS
import torch
import cv2
import numpy as np
from sensor_msgs.msg import Image # Image is the message type
from std_msgs.msg import Int32, Float32, Float32MultiArray, String
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images

# Load custom model
model = torch.hub.load('/home/gilberto/first_ws/src/mine_cam_finder/src/yolov5', 'custom', path = '/home/gilberto/weights/best_v5.pt', force_reload=True, skip_validation=True, source='local')

global bbox

def callback(data): 

  ### Variables ###
  global bbox
  x = 0
  y = 0
  xx = 0
  yy = 0
  conf = 0
  cla = 0


  ### Initializations ###
  br = CvBridge() # Used to convert between ROS and OpenCV images
  img = br.imgmsg_to_cv2(data, "bgr8")
  #rospy.loginfo("receiving video frame") # Output debugging info to the terminal

  # Make detections
  model.conf = 0.75
  # model.max_det = 1
  results = model(img)

  # Retrieve detection info values
  for box in results.xyxy[0]: 
      if box[5] < 3:
          x = int(box[0])
          y = int(box[1])
          xx = int(box[2])    
          yy = int(box[3]) 
          conf = round(float(box[4]), 2)
          cla = int(box[5])
          
  # print(results.xyxy[0])

  # Publishing bounding box values of detected target
  bbox = Float32MultiArray()
  bbox.data = [x, y, xx, yy]
  # print(bbox)
  bound_box.publish(bbox) 

  # Publishing confidence and class values of detected target
  conf_pub.publish(conf)
  class_pub.publish(cla)
  # print(cla, conf)

  # Converting imgYOLO to a readable Image for ROS and publishing
  imgYOLO = np.squeeze(results.render())
  imgYOLOconvert = br.cv2_to_imgmsg(imgYOLO)
  detect_img_pub.publish(imgYOLOconvert)

  # Show camera
  cv2.imshow("Camera", imgYOLO)

  r = rospy.Rate(1)
  r.sleep
  
  cv2.waitKey(1)

def receive_message():
  rospy.init_node('bounding_box')
  rospy.Subscriber('/camera/color/image_raw', Image, callback) # /usb_cam/image_raw
  rospy.spin()
  cv2.destroyAllWindows()

if __name__ == '__main__':
  bound_box = rospy.Publisher('bounding_box', Float32MultiArray, queue_size=1)
  conf_pub = rospy.Publisher('target_conf', Float32, queue_size=1)
  class_pub = rospy.Publisher('target_class', Int32, queue_size=1)
  detect_img_pub = rospy.Publisher('YOLO_img', Image, queue_size=1)
  receive_message()