#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool, Float32, Int32, Float32MultiArray, String
from geometry_msgs.msg import Twist

# Function for logging switch
def switch_mode(data):
    global mode
    # Change the mode based on the value of the button message
    mode = data.data
    # Print a message indicating the new mode
    if mode:
        pass # Switching to autonomous mode 
    else:
        string1 = "Switching to RC"
        status_string_pub.publish(string1)
        print(string1)

# Retrieving receiver data
def receiver_callback(data):
    receiver_input_array = data.data
    # rospy.loginfo("reciever input: " + str(receiver_input_array))
    cmdvel_controller(receiver_input_array)

# Main 'cmd_vel' function
def cmdvel_controller(receiver_input):
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            if mode:
                pass # Driving autonomous mode
            else:
                string2 = "RC mode"
                status_string_pub.publish(string2)
                print(string2)

                left_wheel = receiver_input[0]
                right_wheel = receiver_input[1]

                # determine the movement based on the value of left_wheel and right_wheel
                left_speed = 0
                right_speed = 0
                if left_wheel > 1560:
                    left_speed = (left_wheel - 1500) / 500
                    if left_speed > 0.8:
                        left_speed = 1.0
                elif left_wheel < 1420:
                    left_speed = -(1500 - left_wheel) / 500
                    if left_speed < -0.8:
                        left_speed = -1.0
                if right_wheel > 1560:
                    right_speed = (right_wheel - 1500) / 500
                    if right_speed > 0.8:
                        right_speed = 1.0
                elif right_wheel < 1420:
                    right_speed = -(1500 - right_wheel) / 500
                    if right_speed < -0.8:
                        right_speed = -1.0


                # create and publish the Twist message
                cmd_vel = Twist()
                cmd_vel.linear.x = (left_speed + right_speed) / 2
                cmd_vel.angular.z = (right_speed - left_speed) / 2
                cmd_vel_pub.publish(cmd_vel)

                break

        except NameError:
            string3 = "Select a mode"
            status_string_pub.publish(string3)
            print(string3)
            

        # Sleep to maintain the loop rate
        rate.sleep()

# Topic receiver
def receive_message():
    rospy.init_node("RC_driver")
    rospy.Subscriber("autopilot_check", Bool, switch_mode)
    rospy.Subscriber("reciever_input", Float32MultiArray, receiver_callback)
    rospy.spin()

if __name__ == '__main__':
    cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
    status_string_pub = rospy.Publisher('status_string', String, queue_size=1)
    receive_message()
#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool, Float32, Int32, Float32MultiArray, String
from geometry_msgs.msg import Twist

# Function for logging switch
def switch_mode(data):
    global mode
    # Change the mode based on the value of the button message
    mode = data.data
    # Print a message indicating the new mode
    if mode:
        pass # Switching to autonomous mode 
    else:
        string1 = "Switching to RC"
        status_string_pub.publish(string1)
        print(string1)

# Retrieving receiver data
def receiver_callback(data):
    receiver_input_array = data.data
    # rospy.loginfo("reciever input: " + str(receiver_input_array))
    cmdvel_controller(receiver_input_array)

# Main 'cmd_vel' function
def cmdvel_controller(receiver_input):
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            if mode:
                pass # Driving autonomous mode
            else:
                string2 = "RC mode"
                status_string_pub.publish(string2)
                print(string2)

                left_wheel = receiver_input[0]
                right_wheel = receiver_input[1]

                # determine the movement based on the value of left_wheel and right_wheel
                left_speed = 0
                right_speed = 0
                if left_wheel > 1560:
                    left_speed = (left_wheel - 1500) / 500
                    if left_speed > 0.8:
                        left_speed = 1.0
                elif left_wheel < 1420:
                    left_speed = -(1500 - left_wheel) / 500
                    if left_speed < -0.8:
                        left_speed = -1.0
                if right_wheel > 1560:
                    right_speed = (right_wheel - 1500) / 500
                    if right_speed > 0.8:
                        right_speed = 1.0
                elif right_wheel < 1420:
                    right_speed = -(1500 - right_wheel) / 500
                    if right_speed < -0.8:
                        right_speed = -1.0


                # create and publish the Twist message
                cmd_vel = Twist()
                cmd_vel.linear.x = (left_speed + right_speed) / 2
                cmd_vel.angular.z = (right_speed - left_speed) / 2
                cmd_vel_pub.publish(cmd_vel)

                break

        except NameError:
            string3 = "Select a mode"
            status_string_pub.publish(string3)
            print(string3)
            

        # Sleep to maintain the loop rate
        rate.sleep()

# Topic receiver
def receive_message():
    rospy.init_node("RC_driver")
    rospy.Subscriber("autopilot_check", Bool, switch_mode)
    rospy.Subscriber("reciever_input", Float32MultiArray, receiver_callback)
    rospy.spin()

if __name__ == '__main__':
    cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
    status_string_pub = rospy.Publisher('status_string', String, queue_size=1)
    receive_message()