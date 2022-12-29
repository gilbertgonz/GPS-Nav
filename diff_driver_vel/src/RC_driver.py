#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool, Float32, Int32
from geometry_msgs.msg import Twist
import time

# Define the switch_mode function
def switch_mode(data):
    global mode
    # Change the mode based on the value of the button message
    mode = data.data
    # Print a message indicating the new mode
    if mode:
        rospy.loginfo("Switching to autonomous mode")
    else:
        rospy.loginfo("Switching to remote control mode")

def receiver_callback(data):
    global receiver_input
    receiver_input = data.data

# Initialize the node and the mode variable
rospy.init_node("RC_driver")
mode = False

# Set up subscribers
rospy.Subscriber("autopilot_check", Bool, switch_mode)
rospy.Subscriber("reciever_input", array, receiver_callback)  ################ make a reliable reciever_input node #################

# Set up the velocity publisher
cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

# Set the loop rate
rate = rospy.Rate(10)

while not rospy.is_shutdown():
    # If in autonomous mode, set the twist message to the desired autonomous velocity
    if mode:
        rospy.loginfo("Driving autonomous mode")
    else:
        rospy.loginfo("Driving RC mode")

        twist = Twist()

        # Map the receiver input string to the linear and angular velocities of the twist message
        if receiver_input == '11':
            twist.linear.x = 1.0
            twist.angular.z = 0.0
        elif receiver_input == '00':
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        elif receiver_input == '01':
            twist.linear.x = 0.0
            twist.angular.z = 1.0
        elif receiver_input == '10':
            twist.linear.x = 0.0
            twist.angular.z = -1.0
        else
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        
    # Publish the twist message
    cmd_vel_pub.publish(twist)

    # Sleep to maintain the loop rate
    rate.sleep()