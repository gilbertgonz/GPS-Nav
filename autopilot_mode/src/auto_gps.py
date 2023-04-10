#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import NavSatFix
import math

## TODO: fix the main while loop (in the bottom) so that when 'md' is false, the robot goes back to RC mode

class GPSNav:
    def __init__(self):
        self.twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.curr_coor_sub = rospy.Subscriber('/current_coordinate', NavSatFix, self.curr_coor_callback)
        self.way_coor_sub = rospy.Subscriber('/waypoint_coordinate', NavSatFix, self.way_coor_callback)
        self.mpu_sub = rospy.Subscriber('/compass', Float32, self.mpu_callback)

        self.auto_control_sub = rospy.Subscriber('/gps_mode', Bool, self.mode_callback)

        self.mode = False
        self.waypoint_accuracy = 0.5
        self.wheel_distance = 0.254  # meters
        self.max_speed = 0.35  # meters per second (for calculation purposes)
        self.rate = rospy.Rate(10)

        # Test Waypoint close distance forward: 25.714988523022885, -80.36150001540271
        # Test Waypoint 90deg CW: 25.714955295146392, -80.36090858833603

    def curr_coor_callback(self, data):
        self.curr_lat = data.latitude
        self.curr_long = data.longitude

    def way_coor_callback(self, data):
        self.way_lat = data.latitude
        self.way_long = data.longitude

        self.dist_to_travel, self.desired_angle = self.dist_desang_calc(self.curr_lat, self.curr_long, self.way_lat, self.way_long)

    def mode_callback(self, data):
        self.mode = data.data

    def mpu_callback(self, data):
        self.current_angle = data.data
        
    # Distance and desired angle calculator
    def dist_desang_calc(self, curr_lat, curr_long, way_lat, way_long):
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

    def turn_and_move_robot(self):
        try:
            # print(self.mode)
            if self.mode and self.dist_to_travel > self.waypoint_accuracy:
                # Turning
                rospy.loginfo("Waypoint: %s" % (str(self.way_lat) + ", " + str(self.way_long)))
                rospy.loginfo("Current angle: {}".format(self.current_angle))
                rospy.loginfo("Desired angle: {}".format(self.desired_angle))
                rospy.loginfo("Turning")

                while abs(self.desired_angle - self.current_angle) > 0.5 and self.mode:
                    error = self.desired_angle - self.current_angle

                    if error > 180:
                        error -= 360
                    elif error < -180:
                        error += 360

                    if error > 0:
                        self.twist_pub.publish(Twist(linear=Vector3(0.1, 0, 0), angular=Vector3(0, 0, -0.4)))
                    else:
                        self.twist_pub.publish(Twist(linear=Vector3(0.1, 0, 0), angular=Vector3(0, 0, 0.4)))

                    if not self.mode:
                        break

                # rospy.loginfo("Finished turn")

                # Moving
                dist_traveled = 0.0
                rospy.loginfo("Moving")
                rospy.loginfo("Distance (m) to waypoint: %s", self.dist_to_travel)

                time_to_travel = self.dist_to_travel / self.max_speed

                self.twist_pub.publish(Twist(linear=Vector3(0.15, 0, 0), angular=Vector3(0, 0, 0)))
                
                while dist_traveled < self.dist_to_travel and self.mode:
                    dist_traveled += self.max_speed * self.rate.sleep_dur.to_sec()
                    self.rate.sleep()
                    if not self.mode:
                        break

                rospy.loginfo("Arrived")

            else:
                # self.twist_pub.publish(Twist(linear=Vector3(0, 0, 0), angular=Vector3(0, 0, 0)))
                return

        except AttributeError:
            rospy.loginfo("Enter a waypoint")
            self.twist_pub.publish(Twist(linear=Vector3(0, 0, 0), angular=Vector3(0, 0, 0)))


if __name__ == "__main__":
    rospy.init_node('auto_gps')
    node = GPSNav()
    md = False

    def mode_cb(data):
        global md 
        md = data.data

    rospy.Subscriber('/gps_mode', Bool, mode_cb)

    rate = rospy.Rate(10)  # set a rate of 10 Hz

    while not rospy.is_shutdown():
        # print(md)
        if md:
            node.turn_and_move_robot()
        rate.sleep()

    rospy.spin()
