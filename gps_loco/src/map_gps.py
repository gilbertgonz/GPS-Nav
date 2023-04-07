#!/usr/bin/env python3

import rospy
import folium
import datetime
#!/usr/bin/env python3

import rospy
import folium
import datetime
import argparse

class MapNode:
    def __init__(self, filename):
        self.map = None
        self.prev_latitude = None
        self.prev_longitude = None
        self.filename = filename
        self.timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.map_filename = f'/home/first_ws/src/gps_loco/src/maps/map_{self.timestamp}.html'
        self.prev_num_lines = 0

    def generate_map(self):
        num_lines = sum(1 for line in open(self.filename))
        if num_lines <= self.prev_num_lines:
            return
        
        with open(self.filename) as file:
            lines = file.readlines()     
        for line in lines:
            data = line.strip().split(",")
            latitude = float(data[0])
            longitude = float(data[1])

            if self.map is None:
                self.map = folium.Map(location=[float(latitude), float(longitude)], zoom_start=100)

            # If the data is the current coordinates of the robot
            if len(data) == 2:
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
        self.prev_num_lines = num_lines

    def run(self):
        rate = rospy.Rate(1)  # 1 Hz
        while not rospy.is_shutdown():
            try:
                self.generate_map()
                # rospy.loginfo("host name Map generated.")
            except FileNotFoundError:
                rospy.loginfo("GPS data file not found.")
            rate.sleep()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-f', '--filename', type=str, default='/home/first_ws/src/gps_loco/src/txt_files/miami_data.txt', help='GPS data file name')
    args = parser.parse_args()
    
    rospy.init_node('map_node')
    map_node = MapNode(args.filename)
    map_node.run()