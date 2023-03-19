# GPS-Nav
This ROS package was developed with the intent to provide GPS-waypoint based autonomous navigation for an Autonomous Vehicle for Detecting Landmines (AVDL); my senior year Engineering project at Florida International University (FIU). It consists of a GUI where the user can see the current location of the robot, location of detected landmines, input waypoint locations, and buttons that the user can utlize to control the robot.

# Dependencies
Tested on:

- Ubuntu 20.04 (ROS Noetic)

**Open Source Packages**

- common_msgs // ensures you have all message type dependencies

- imu_tools // for extracting IMU data from Intel RealSense 435i

- realsense-ros // driver for RealSense cameras

- rplidar ros // driver for RPLiDARs

- rtabmap_ros // for generating 3D map and visual odometry data using RealSense camera

- roboclaw // driver for utlizing RoboClaw 2x15a motor controller

- move_base // ROS navigation package

## 1. Installation
Ensure ROS Noetic and all previously mentioned dependencies are correctly installed in a Ubuntu 20.04 system
### 1.1 Clone and install:

    sudo apt install -y python-rosdep
    cd <your_ws>/src
    git clone https://github.com/gilbertgonz/GPS-Nav.git
    cd ..
    rosdep install --from-paths src --ignore-src -r -y

### 1.2 Build your workspace:

    cd <your_ws>
    catkin_make
    source <your_ws>/devel/setup.bash
## 2. Quick Start

You don't need a physical robot to visualize the GUI. If you're building a physical robot, you can find out more how to configure and run a new robot in [ROS Wiki Tutorials](http://wiki.ros.org/ROS/Tutorials).

### 2.1 Run the main launch file:

    roslaunch gps_loco control_minefinder.launch
    
# User Interface
Note that the application of this project is to control an AVDL, thus there are certain functions in the UI that may not be relevant other AVs of different applications. However, `customtkinter` is a very popular and well-documented GUI API for Python 3 that was used to develop this UI; therefore altering the code for this UI can easily be done to better fit other applications.

![image](https://user-images.githubusercontent.com/110450734/226206246-c204ac5e-e70d-4b0d-b520-d5b59ac275b2.png)
