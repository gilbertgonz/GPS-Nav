#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, Int32
from sensor_msgs.msg import NavSatFix
import math

lat1, long1, lat2, long2 = 0, 0, 0, 0

def callback(data):
    global lat1, long1

    lat1 = data.latitude
    long1 = data.longitude

def callback1(data):
    global lat2, long2

    gps_distance_pub = rospy.Publisher('distance_to_waypoint', Float32, queue_size=1)

    lat2 = data.latitude
    long2 = data.longitude
    rospy.loginfo("Waypoint: %s" % (str(lat2) + ", " + str(long2)))

    d = distance_calculator(lat1, long1, lat2, long2)

    connections = gps_distance_pub.get_num_connections()
    if connections > 0:  # ensure there is connection before continuing
        gps_distance_pub.publish(d) # publishing distance value
        rospy.loginfo("Distance (m) from waypoint: %s", d)
        rospy.loginfo("Angle (deg) change: %s", ang_change)

def mpu_callback(data):
    global ang_change
    mpu_angle = data.data

    ang_change_pub = rospy.Publisher('direction_angle', Float32, queue_size=1)

    ang_change = angle_calculator(mpu_angle)

    connections = ang_change_pub.get_num_connections()
    if connections > 0:  # ensure there is connection before continuing
        ang_change_pub.publish(ang_change) # publishing distance value

def distance_calculator(lat1, long1, lat2, long2):
    R = 6371 # earth radius in kilometers

    Lat1 = lat1 * math.pi/180 # convert to radian
    Lat2 = lat2 * math.pi/180

    Lat = (lat2-lat1) * math.pi/180  # find change in respective coordinates
    long = (long2-long1) * math.pi/180

    a = math.sin(Lat/2) * math.sin(Lat/2) + math.cos(Lat1) * math.cos(Lat2) * math.sin(long/2) * math.sin(long/2)  # Haversine Formula
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

    d = round((R * c)*1000, 3) # distance in meters

    return d

def angle_calculator(current_angle):
    lat_diff = float(lat2 - lat1)
    long_diff = float(long2 - long1)

    #print(lat2, long2)
    #print(long_diff)
    #print(lat_diff)

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

    # print(ang)       
    # print(dir_ang)
    # ang_change = round(dir_ang - current_angle, 1)
    ang_change = int(dir_ang - current_angle)

    return dir_ang
    
def receive_message():
    rospy.init_node('gps_calculator')

    rospy.Subscriber('/current_coordinate', NavSatFix, callback, queue_size=1)
    rospy.Subscriber('/waypoint_coordinate', NavSatFix, callback1, queue_size=1)
    rospy.Subscriber('/compass', Float32, mpu_callback, queue_size=1)

    rospy.spin()

if __name__== '__main__':
    receive_message()