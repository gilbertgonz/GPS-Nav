# GPS-Nav
This ROS package was developed with the intent to provide GPS-waypoint based autonomous navigation for a landmine detecting AV. It consists of a GUI where the user can see the current location of the robot, location of detected landmines, input waypoint locations, and buttons that the user can utlize to control the robot.

# Dependent Packages
Tested in ROS Noetic and Ubuntu 20.04

common_msgs // ensures you have all message type dependencies

imu_tools // for extracting IMU data from Intel RealSense 435i

realsense-ros // driver for RealSense cameras

rplidar ros // driver for RPLiDARs

rtabmap_ros // for generating 3D map and visual odometry data using RealSense camera

move_base // ROS navigation package
