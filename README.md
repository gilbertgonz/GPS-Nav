# GPS-Nav
This ROS package was developed with the intent to provide GPS-waypoint based autonomous navigation for an Autonomous Vehicle for Detecting Landmines (AVDL); my senior year Engineering project at Florida International University (FIU). It consists of a GUI where the user can see the current location of the robot, location of detected landmines, input waypoint locations, and buttons that the user can utlize to control the robot.

# Dependencies
**Tested on:**

- Ubuntu 20.04 (ROS Noetic)

**Open Source Packages**

- common_msgs // ensures you have all message type dependencies

- gazebo_ros_pkgs // ensures you have all dependencies for Gazebo-ROS interactiosn

- turtlebot3_simulations // package for turtlebot3 Gazebo simulations

- imu_tools // for extracting IMU data from Intel RealSense 435i

- realsense-ros // driver for RealSense cameras

- rplidar ros // driver for RPLiDARs

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
    
# User Interface (How it works)
Firstly, it is evident that the top section of the UI consists of three self-explanatory sets of text entries: current coordinate, waypoint coordinate, and home coordinate. The 'Current Coordinate' section displays the current coordinates of the robot whenever the GPS locks signal with a satellite. The text box on the left titled 'Detected Mines' is where all detected mines are logged. Whenever there is a certain threshold of metal detected or the computer vision system detects what may look like a landmine, also at a certain accuracy threshold, the current coordinates along with a short text of what was detected will be logged into the text box. The buttons catalog on the right consists of the main controls for the robot. The 'Autopilot' button begins execution of a landmine-detecting autopilot mission, where all that is needed is a set of waypoint coordinates that can be registered by entering the waypoint coordinates in their respective text boxes and clicking the 'New Waypoint' button. The 'Return Home' button initiates a similar function as the 'Autopilot' button; however, the home coordinates would be entered in its respective text entry ("Home Coordinate"). The 'Image Capture' button is pretty self-explanatory; it returns an image to the user of what the robot is seeing at any point in time, displaying the current GPS location of the robot on the top left, as well as any landmines that it may detect through the computer vision system (please refer to the next image figure for reference). Lastly, the 'Stop/RC' button is the emergency stop button that stops the robot of any autopilot missions and immediately initiates remote controlled mode.

Note that the application of this project is to control an AVDL, thus there are certain functions in the UI that may not be relevant other AVs of different applications. However, `customtkinter` is a very popular and well-documented GUI API for Python 3 that was used to develop this UI; therefore altering the code for this UI can easily be done to better fit other applications.

![image](https://user-images.githubusercontent.com/110450734/226206246-c204ac5e-e70d-4b0d-b520-d5b59ac275b2.png)
