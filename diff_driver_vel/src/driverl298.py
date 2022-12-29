#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Int32, Bool
import RPi.GPIO as GPIO          
import time

GPIO.setmode(GPIO.BCM)
GPIO.setup(17,GPIO.OUT)
GPIO.setup(22,GPIO.OUT)
GPIO.setup(23,GPIO.OUT)
GPIO.setup(24,GPIO.OUT)

def callback(data):
    linear = data.linear.x
    angular = data.angular.z

    if linear > 0:
        rospy.loginfo("Forward")
        GPIO.output(17,GPIO.LOW)
        GPIO.output(22,GPIO.HIGH)
        GPIO.output(23,GPIO.LOW)
        GPIO.output(24,GPIO.HIGH)
    elif linear < 0:
        rospy.loginfo("Reverse")
        GPIO.output(17,GPIO.HIGH)
        GPIO.output(22,GPIO.LOW)
        GPIO.output(23,GPIO.HIGH)
        GPIO.output(24,GPIO.LOW)
    elif angular < 0:
        rospy.loginfo("Right")
        GPIO.output(17,GPIO.LOW)
        GPIO.output(22,GPIO.HIGH)
        GPIO.output(23,GPIO.HIGH)
        GPIO.output(24,GPIO.LOW)
    elif angular > 0:
        rospy.loginfo("Left")
        GPIO.output(17,GPIO.HIGH)
        GPIO.output(22,GPIO.LOW)
        GPIO.output(23,GPIO.LOW)
        GPIO.output(24,GPIO.HIGH)
    elif angular == 0 and linear == 0:
        rospy.loginfo("Stop")
        GPIO.output(17,GPIO.LOW)
        GPIO.output(22,GPIO.LOW)
        GPIO.output(23,GPIO.LOW)
        GPIO.output(24,GPIO.LOW)

def receive_message():
    rospy.init_node('L298_driver')  # Initializing node
    rospy.Subscriber('/cmd_vel', Twist, callback, queue_size=1)  # Subscribing to teleop
    rospy.spin()

if __name__== '__main__':
    receive_message()