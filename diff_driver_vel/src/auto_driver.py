#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool, Float32, Int32
from geometry_msgs.msg import Twist
import time
import math

# Function for logging switch
def switch_mode(data):
    global mode
    mode = data.data
    if mode:
        rospy.loginfo("Switching to autonomous mode")
    else:
        rospy.loginfo("Switching to remote control mode")
# Mode variable retrieving function
def autopilot(data):
    global distance
    distance = data.data 
# Total turn time calculator function
def angle_data(data):
    global wheel_speed, total_time
    ang = data.data

    # Convert the angle to radians
    angle_radians = ang * math.pi / 180

    # Define the constants
    wheel_speed = 0.4124  # m/s
    wheel_diameter = 0.065278  # meters
    base_width = 0.13462  # meters

    # Calculate the circumference of the wheels
    wheel_circumference = math.pi * wheel_diameter

    # Calculate the distance traveled by each wheel during the turn
    turn_distance = base_width / 2 * angle_radians

    # Calculate the duration of the turn
    total_time = turn_distance / wheel_speed

# Initialize node and mode variable
rospy.init_node("auto_driver")
mode = False

# Set up subscribers
rospy.Subscriber("autopilot_check", Bool, switch_mode)
rospy.Subscriber('angle_change', Int32, angle_data, queue_size=1)
rospy.Subscriber('distance_to_waypoint', Float32, autopilot, queue_size=1)

# Set up /cmd_vel publisher
cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

# Start main loop
rate = rospy.Rate(10)
while not rospy.is_shutdown():
    if mode:  # If mode == True, execute autonomous drive
        rospy.loginfo("Driving autonomous mode")
        twist = Twist()  # Initialize twist messages

        rospy.loginfo("turn time: " + str(total_time))  # logging turn time

        start_time = time.time()
        while time.time() - start_time <= total_time:  # Executing auto turn per calculated turn time from angle_data()
            # Turn right (clockwise)
            twist.linear.x = 0.0
            twist.angular.z = -1.0
            cmd_vel_pub.publish(twist)
            if not mode:  # Break loop if RC mode is initiated
                break
        rospy.loginfo("finished turn")  # Logging finished turn

        # duration of travel
        distance_duration = round(distance / wheel_speed, 3)
        rospy.loginfo("distance recieved: " + str(distance)) 
        rospy.loginfo("duration: " + str(distance_duration)) 

        start_time2 = time.time()
        while time.time() - start_time2 <= distance_duration:  # Executing auto drive per calculated distance duration
            # Drive forward
            twist.linear.x = 1.0
            twist.angular.z = 0.0
            cmd_vel_pub.publish(twist)
            if not mode:  # Break loop if RC mode is initiated
                break
        rospy.loginfo("finished auto drive")  # Logging finished auto drive

    else:
        rospy.loginfo("Driving RC mode")  

    # Sleep to maintain the loop rate
    rate.sleep()
