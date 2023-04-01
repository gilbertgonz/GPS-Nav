#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import NavSatFix
import math
import time

curr_lat = 0
curr_long = 0
way_lat = 0
way_long = 0
current_angle = 0
mode = False
waypoint_accuracy = 0.5
wheel_distance = 0.254  # meters
max_speed = 0.35  # meters per second (for calculation purposes)

# Test Waypoint close distance forward: 25.714988523022885, -80.36150001540271
# Test Waypoint 90deg CW: 25.714955295146392, -80.36090858833603

def curr_coor_callback(data):
    global curr_lat, curr_long
    curr_lat = data.latitude
    curr_long = data.longitude

def way_coor_callback(data):
    global way_lat, way_long, dist_to_travel, desired_angle
    way_lat = data.latitude
    way_long = data.longitude

    dist_to_travel, desired_angle = dist_desang_calc(curr_lat, curr_long, way_lat, way_long)

def mode_callback(data):
    global mode
    mode = data.data

def mpu_callback(data):
    global current_angle
    current_angle = data.data

# Distance and desired angle calculator
def dist_desang_calc(curr_lat, curr_long, way_lat, way_long):
    R = 6371 # earth radius in kilometers

    Lat1 = curr_lat * math.pi/180 # convert to radian
    Lat2 = curr_long * math.pi/180

    lat_diff = float(way_lat - curr_lat)
    long_diff = float(way_long - curr_long)

    Lat = (way_lat-curr_lat) * math.pi/180  # find change in respective coordinates
    long = (way_long-curr_long) * math.pi/180

    a = math.sin(Lat/2) * math.sin(Lat/2) + math.cos(Lat1) * math.cos(Lat2) * math.sin(long/2) * math.sin(long/2)  # Haversine Formula
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

    d = round((R * c)*1000, 3) # distance in meters

    ang = 0
    if lat_diff != 0.0:
        ang = math.atan(long_diff/lat_diff)
        ang = abs(ang * 180/math.pi)

    if long_diff >= 0 and lat_diff >= 0:
        dir_ang = ang
    if long_diff >= 0 and lat_diff < 0:
        dir_ang = 90 + (90 - ang)
    if long_diff < 0 and lat_diff < 0:
        dir_ang = 180 + ang
    if long_diff < 0 and lat_diff >= 0:
        dir_ang = 359 - ang
    if lat_diff == 0.0:
            if long_diff > 0:
                dir_ang = 270
            else:
                dir_ang = 90 
    if long_diff == 0.0:
            if lat_diff > 0:
                dir_ang = 0
            else:
                dir_ang = 180 

    return d, dir_ang

rospy.init_node("auto_gps")
mode = False

curr_coor_sub = rospy.Subscriber('/current_coordinate', NavSatFix, curr_coor_callback)
way_coor_sub = rospy.Subscriber('/waypoint_coordinate', NavSatFix, way_coor_callback)
mpu_sub = rospy.Subscriber('/compass', Float32, mpu_callback)
auto_control_sub = rospy.Subscriber('/gps_mode', Bool, mode_callback)

twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

rate = rospy.Rate(10)
while not rospy.is_shutdown():
    try:
        if mode and dist_to_travel > waypoint_accuracy:
            # Turning
            rospy.loginfo("Waypoint: %s" % (str(way_lat) + ", " + str(way_long)))
            rospy.loginfo("Current angle: {}".format(current_angle))
            rospy.loginfo("Desired angle: {}".format(desired_angle))
            rospy.loginfo("Turning")

            while abs(desired_angle - current_angle) > 0.5 and mode:
                error = desired_angle - current_angle

                if error > 0:
                    twist_pub.publish(Twist(linear=Vector3(0, 0, 0), angular=Vector3(0, 0, -0.25)))
                else:
                    twist_pub.publish(Twist(linear=Vector3(0, 0, 0), angular=Vector3(0, 0, 0.25)))
                
                if not mode:
                    break

            # twist_pub.publish(Twist(linear=Vector3(0, 0, 0), angular=Vector3(0, 0, 0)))

            rospy.loginfo("Finished turn")

            # Moving
            dist_traveled = 0.0
            rospy.loginfo("Moving")
            rospy.loginfo("Distance (m) to waypoint: %s", dist_to_travel)

            time_to_travel = dist_to_travel / max_speed
            
            start_time = time.time()
            while time.time() - start_time <= time_to_travel:
                twist_pub.publish(Twist(linear=Vector3(0.15, 0, 0), angular=Vector3(0, 0, 0)))
                if not mode:
                    break
            
            # twist_pub.publish(Twist(linear=Vector3(0, 0, 0), angular=Vector3(0, 0, 0)))

            rospy.loginfo("Arrived")

        else:
            pass

    except NameError:
        rospy.loginfo("Enter a waypoint")
        twist_pub.publish(Twist(linear=Vector3(0, 0, 0), angular=Vector3(0, 0, 0)))

# rospy.spin()

    
