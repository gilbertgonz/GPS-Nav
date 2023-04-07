#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from flask import Flask, render_template, Response

app = Flask(__name__)
bridge = CvBridge()
latest_image = None

@app.route('/')
def index():
    return '''<!DOCTYPE html>
              <html>
              <head>
                  <title>AVDL Camera Feed</title>
              </head>
              <body>
                  <h1>AVDL Camera Feed</h1>
                  <img src="/avdl" width="1280" height="960">
              </body>
              </html>'''

def image_callback(msg):
    global latest_image
    latest_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

def generate_image():
    global latest_image
    while not rospy.is_shutdown():
        if latest_image is not None:
            _, buffer = cv2.imencode('.jpg', latest_image)
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')

@app.route('/avdl')
def video_feed():
    return Response(generate_image(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

def run():
    rospy.init_node('webcam_server')
    rospy.Subscriber('/usb_cam/image_raw', Image, image_callback)
    app.run(host='0.0.0.0', port=8080)

if __name__ == '__main__':
    run()