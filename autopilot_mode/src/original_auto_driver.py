#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool, Float32, Int32, String
from geometry_msgs.msg import Twist
import time
import math

# Function for logging switch
def switch_mode(data):
    global mode
    mode = data.data
    if mode:
        string = "Switching to Auto"
        status_string_pub.publish(string)
        print(string)
    else:
        pass # Switching to remote control mode

# Distance retrieving function
def distance_callback(data):
    global distance
    distance = data.data 

# Total turn time calculator function
def angle_data(data):
    global wheel_speed, total_time
    ang = data.data

    # Convert the angle to radians
    angle_radians = ang * math.pi / 180

    # Define the constants
    wheel_speed = 0.5  # m/s
    base_width = 0.3048  # meters

    # Calculate the distance traveled by each wheel during the turn
    turn_distance = base_width / 2 * angle_radians

    # Calculate the duration of the turn
    total_time = (turn_distance / wheel_speed)

# Initialize node and mode variable
rospy.init_node("auto_driver")
mode = False

# Set up subscribers
rospy.Subscriber("autopilot_check", Bool, switch_mode)
rospy.Subscriber('angle_change', Int32, angle_data, queue_size=1)
rospy.Subscriber('distance_to_waypoint', Float32, distance_callback, queue_size=1)

# Set up publishers
cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
status_string_pub = rospy.Publisher('status_string', String, queue_size=1)


# Start main loop
rate = rospy.Rate(10)
prev_totaltime = 0
while not rospy.is_shutdown():
    try:
        if mode and distance > 0:
            string = "Autopilot..."
            status_string_pub.publish(string)
            print(string)

            twist = Twist()  # Initialize twist messages

            if prev_totaltime != total_time:
                rospy.loginfo("turn time: " + str(total_time))  # logging turn time
                prev_totaltime = total_time

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

            rospy.loginfo("distance to waypoint: " + str(distance))
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
            pass # Driving RC mode

    except NameError:  # Error handling
        string = "Enter a waypoint"
        status_string_pub.publish(string)
        print(string)

    # Sleep to maintain the loop rate
    rate.sleep()
