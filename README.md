# Table of Contents
<details>
<summary>Click to expand</summary>

   * [What is this?](#what-is-this)
   * [Requirements](#requirements)
   * [How it works](#how-it-works)
      * [Security](#security)
      * [Mine Logging](#mine-logging)
      * [Buttons](#buttons)
      * [User Interface](#user-interface)
   * [How to use](#how-to-use)
   * [Demo](#demo)
   * [Docker](#docker)
  
</details>

# What is this?

This repository holds the code to my undergraduate senior design project: a landmine-detecting robot. Complete integration, from hardware to software. It uses machine learning for landmine detection and lidar for basic navigation. It also features a GUI showing robot location, location of detected landmines, waypoints, and buttons to control the robot.

<p align="center">
  <img src="https://github.com/gilbertgonz/GPS-Nav/blob/main/assets/avdl1.jpg">
</p>

Watch demo here:
[AVDL Demo](https://youtu.be/Mn7AGbWDKaQ?si=-d_RT7CEDXo3FswM)

# Requirements
**Tested on:**

- Ubuntu 20.04 (ROS Noetic)

**Open Source Packages**

- common_msgs

- gazebo_ros_pkgs

- turtlebot3_simulations

- imu_tools

- realsense-ros

- rplidar_ros 

- folium

- customtkinter

# How it works

## Security

<p align="center">
  <img width="320" height="150" src="https://github.com/gilbertgonz/GPS-Nav/blob/main/assets/sequrity.png?raw=true">
</p>

When the main launch file is ran (see the [How to use](#how-to-use) section), a simple security wall is generated and requires a password for all of the ROS nodes to completely launch. The default password is '1', but it can be changed in the `secured_control_center.py` file.

## User Interface

<p align="center">
  <img width="600" height="380" src="https://github.com/gilbertgonz/GPS-Nav/blob/main/assets/6in_steel_sand.png?raw=true">
</p>

Note that the application of this project is to control an AVDL, thus there are certain functions in the UI that may not be relevant other AVs of different applications. However, `customtkinter` is a very popular and well-documented GUI API for Python 3 that was used to develop this UI; therefore altering the code for this UI can easily be done to better fit other applications.

## Mine Logging

The top section of the UI consists of three self-explanatory sets of text entries: current coordinate, waypoint coordinate, and home coordinate. The 'Current Coordinate' section displays the current coordinates of the robot whenever the GPS locks signal with a satellite. 

The text box on the left titled 'Detected Mines' is where all detected mines are logged. Whenever there is a certain threshold of metal detected or the computer vision system detects what may look like a landmine, also at a certain accuracy threshold, the current coordinates along with a short text of what was detected will be logged into the text box. 

## Buttons

The buttons catalog on the right consists of the main controls for the robot. The 'Autopilot' button begins execution of a landmine-detecting autopilot mission, where all that is needed is a set of waypoint coordinates that can be registered by entering the waypoint coordinates in their respective text boxes and clicking the 'New Waypoint' button. While the robot is navigating, a GPS satellite map is generated in real-time showcasing the path of the robot and positions of detected landmines:

<p align="center">
  <img width="600" height="350" src="https://github.com/gilbertgonz/GPS-Nav/blob/main/assets/map_sample.png?raw=true">
</p>

The 'Return Home' button initiates a similar function as the 'Autopilot' button; however, the home coordinates would be entered in its respective text entry ("Home Coordinate"). The 'Image Capture' button is pretty self-explanatory; it returns an image to the user of what the robot is seeing at any point in time, displaying the current GPS location of the robot on the top left, as well as any landmines that it may detect through the computer vision system: 

<p align="center">
  <img width="580" height="350" src="https://github.com/gilbertgonz/GPS-Nav/blob/main/assets/cv_img.png?raw=true">
</p>

Lastly, the 'Stop/RC' button is the emergency stop button that stops the robot of any autopilot missions and immediately initiates remote controlled mode.

# How to use
Ensure ROS Noetic and all previously mentioned dependencies are correctly installed in a Ubuntu 20.04 system or docker (see the [Docker](#docker) section)
### 1. Clone and install:

    sudo apt install -y python-rosdep
    cd <your_ws>/src
    git clone https://github.com/gilbertgonz/GPS-Nav.git
    cd ..
    rosdep install --from-paths src --ignore-src -r -y

### 2. Build your workspace:

    cd <your_ws>
    catkin_make
    source <your_ws>/devel/setup.bash

### 3. Run the turtlebot3 simulation:

    roslaunch turtlebot3_gazebo gazebo.launch

### 4. Run the main launch file:

    roslaunch gps_loco control_minefinder.launch
    


# Demo
![](assets/final_official_demo.gif)

# Docker
I have also created an open source docker image containing all the required files/packages. You can use the following command to pull the docker image:

    docker pull gilbertgonzalezz/avdl_docker:1.0
    
Start the Docker container by running:

    # enable access to xhost from the container
    xhost +

    # Run docker and open bash shell
    docker run -it --privileged \
    --env=LOCAL_USER_ID="$(id -u)" \
    -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
    -e DISPLAY=:0 \
    --network host \
    --name=AVDL_container gilbertgonzalezz/avdl_docker:1.0 bash

