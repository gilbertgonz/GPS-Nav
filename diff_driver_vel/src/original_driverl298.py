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

total_time = 1.0
loop_check = 0

def teleop(data):
    rospy.loginfo("Driving RC mode")
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

def autopilot(data, arg1=None):
    distance = data.data 

    while loop_check < 1:
        rospy.loginfo("Driving autonomous mode")
        while True:
            GPIO.output(17,GPIO.LOW)
            GPIO.output(22,GPIO.HIGH)
            GPIO.output(23,GPIO.HIGH)
            GPIO.output(24,GPIO.LOW)

            time.sleep(total_time)
            print("finished turn")
            break

        # duration of travel
        distance_duration = round(distance / wheel_speed, 3)
        print("distance recieved: " + str(distance)) 
        print("duration: " + str(distance_duration)) 

        while True:
            GPIO.output(17,GPIO.LOW)
            GPIO.output(22,GPIO.HIGH)
            GPIO.output(23,GPIO.LOW)
            GPIO.output(24,GPIO.HIGH)

            time.sleep(distance_duration)
            break
        
        print("finished drive")
        loop_check += 1

def angle_data(data):
    global wheel_speed
    ang = data.data

    # Set the speed of the wheels in meters per second
    wheel_speed = 0.4124

    # Set the diameter of the wheels in inches
    wheel_diameter = 2.57

    # Set the width of the base in inches
    base_width = 5.3

    # Calculate the circumference of the wheels in inches
    wheel_circumference = wheel_diameter * 3.14159

    # Calculate the distance traveled by one wheel in one revolution in inches
    distance_per_revolution = wheel_circumference

    # Calculate the distance traveled by both wheels in one revolution in inches
    distance_per_revolution_total = distance_per_revolution * 2

    # Calculate the distance traveled by both wheels in one revolution in meters
    distance_per_revolution_total_meters = distance_per_revolution_total * 0.0254

    # Calculate the time it takes for one revolution at the given speed
    time_per_revolution = distance_per_revolution_total_meters / wheel_speed

    # Calculate the distance traveled by one wheel during the turn in meters
    distance_per_wheel = (base_width / 2) * 0.0254

    # Get the turn angle from angle_data function
    turn_angle = ang

    # Calculate the angle of the turn in radians
    turn_angle_radians = turn_angle * 3.14159 / 180

    # Calculate the distance traveled by one wheel during the turn in meters
    distance_per_wheel_turn = distance_per_wheel * turn_angle_radians / (2 * 3.14159)

    # Calculate the number of revolutions needed to turn the desired angle
    revolutions_needed = distance_per_wheel_turn / distance_per_revolution_total_meters

    # Calculate the total time needed to turn the desired angle
    total_time = time_per_revolution * revolutions_needed
    
    autopilot(data, arg1=total_time)

def control_check(data):
    global concheck
    concheck = data.data
    rospy.loginfo(concheck)

    if concheck:
        rospy.Subscriber('angle_change', Int32, angle_data, queue_size=1)
        rospy.Subscriber('distance_to_waypoint', Float32, autopilot, queue_size=1)
    else:
        rospy.Subscriber('/cmd_vel', Twist, teleop, queue_size=1)  # Subscribing to teleop

def receive_message():
    rospy.init_node('OG_L298_driver')  # Initializing node
    rospy.Subscriber('autopilot_check', Bool, control_check, queue_size=1)
    rospy.spin()

if __name__== '__main__':
    receive_message()