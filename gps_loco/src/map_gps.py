#!/usr/bin/env python3

import rospy
import folium
import datetime
import os
import time

class MapNode:
    def __init__(self):
        self.map = None
        self.prev_latitude = None
        self.prev_longitude = None
        self.filename = None
        self.timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.map_filename = f'/home/gilberto/first_ws/src/gps_loco/src/maps/map_{self.timestamp}.html'
        self.prev_num_lines = 0

    def generate_map(self, event):
        try:
            if self.filename is None:
                # find the latest generated file with .txt extension in the current working directory
                files = os.listdir("/home/gilberto/first_ws/src/gps_loco/src/txt_files")
                txt_dir = '/home/first_ws/src/gps_loco/src/txt_files/'
                txt_files = [f for f in files if f.endswith(".txt")]
                if txt_files:
                    # most_recent_file = max(txt_files, key=os.path.getctime)
                    # self.filename = os.path.join(txt_dir, most_recent_file)
                    self.filename = '/home/gilberto/first_ws/src/gps_loco/src/txt_files/test3.txt'
                else:
                    return

            # num_lines = sum(1 for line in open(self.filename))
            # if num_lines <= self.prev_num_lines:
            #     return

            with open(self.filename) as file:
                lines = file.readlines()     
            for line in lines:
                data = line.strip().split(",")
                latitude = float(data[0])
                longitude = float(data[1])

                if self.map is None:
                    self.map = folium.Map(location=[float(latitude), float(longitude)], zoom_start=1000)

                # If the data is the current coordinates of the robot
                if latitude and longitude:  # if len(data) == 2:
                    # Check if there are already previous coordinates to connect with
                    if self.prev_latitude and self.prev_longitude:
                        # Draw a line between the previous and current coordinates
                        folium.PolyLine(locations=[(self.prev_latitude, self.prev_longitude), (latitude, longitude)], color='blue').add_to(self.map)
                    # Set the current coordinates as previous for the next iteration
                    self.prev_latitude = latitude
                    self.prev_longitude = longitude
                else:  # The data represents a detected mine
                    # Add a marker with a pop-up label
                    folium.Marker(location=[latitude, longitude], popup='Detected mine', icon=folium.Icon(color='red')).add_to(self.map)

            self.map.save(self.map_filename)
            # self.prev_num_lines = num_lines

        except FileNotFoundError:
            rospy.loginfo("GPS data file not found.")
            return

    def run(self):
        rate = rospy.Rate(1)  # set the rate to 1 Hz
        while not rospy.is_shutdown():
            self.generate_map(None)
            rate.sleep()
        # # self.generate_map(None)
        # rospy.Timer(rospy.Duration(1), self.generate_map) # generate_map will be called every 1 seconds
        # rospy.spin() # Keep the program running until ROS is shut down

if __name__ == '__main__':  
    rospy.init_node('map_node')
    map_node = MapNode()
    # time.sleep(1)
    map_node.run()

'''
#!/usr/bin/env python3

import rospy
import folium
import datetime
import os
import time

class MapNode:
    def __init__(self):
        self.map = None
        self.prev_latitude = None
        self.prev_longitude = None
        self.filename = None
        self.timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.map_filename = f'/home/gilberto/first_ws/src/gps_loco/src/maps/map_{self.timestamp}.html'
        self.prev_num_lines = 0
        self.last_mod_time = None

    def generate_map(self, event):
        try:
            if self.filename is None:
                # find the latest generated file with .txt extension in the current working directory
                files = os.listdir("/home/gilberto/first_ws/src/gps_loco/src/txt_files")
                txt_dir = '/home/first_ws/src/gps_loco/src/txt_files/'
                txt_files = [f for f in files if f.endswith(".txt")]
                if txt_files:
                    # most_recent_file = max(txt_files, key=os.path.getctime)
                    # self.filename = os.path.join(txt_dir, most_recent_file)
                    self.filename = '/home/gilberto/first_ws/src/gps_loco/src/txt_files/test2.txt'
                else:
                    return

            num_lines = sum(1 for line in open(self.filename))
            if num_lines <= self.prev_num_lines:
                return
            
            with open(self.filename) as file:
                lines = file.readlines()     
            for line in lines:
                data = line.strip().split(",")
                latitude = float(data[0])
                longitude = float(data[1])
                
            # mod_time = os.path.getmtime(self.filename)
            # if self.last_mod_time is not None and mod_time == self.last_mod_time:
            #     return
            # self.last_mod_time = mod_time
   
            # with open(self.filename, 'r') as file:
            #     last_line = file.readlines()[-1]
            # data = last_line.strip().split(",")
            # latitude = float(data[0])
            # longitude = float(data[1])

            if self.map is None:
                self.map = folium.Map(location=[float(latitude), float(longitude)], zoom_start=1000)

            # If the data is the current coordinates of the robot
            if len(data) == 2:
                # Check if there are already previous coordinates to connect with
                if self.prev_latitude and self.prev_longitude:
                    # Draw a line between the previous and current coordinates
                    folium.PolyLine(locations=[(self.prev_latitude, self.prev_longitude), (latitude, longitude)], color='blue').add_to(self.map)
                # Set the current coordinates as previous for the next iteration
                self.prev_latitude = latitude
                self.prev_longitude = longitude
            else:
                # Add a marker with a pop-up label
                folium.Marker(location=[latitude, longitude], popup='Detected mine', icon=folium.Icon(color='red')).add_to(self.map)

            self.map.save(self.map_filename)
            self.prev_num_lines = num_lines

        except FileNotFoundError:
            rospy.loginfo("GPS data file not found.")
            return

    def run(self):
        # self.timer = rospy.Timer(rospy.Duration(3), self.generate_map) # generate_map will be called every 0.05 seconds
        self.generate_map(None)
        rospy.spin() # Keep the program running until ROS is shut down

if __name__ == '__main__':  
    rospy.init_node('map_node')
    map_node = MapNode()
    map_node.run()'''