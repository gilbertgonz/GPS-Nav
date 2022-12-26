#!/usr/bin/env python3

import rospy
from array_msgs.msg import array
from std_msgs.msg import Float32, Int32
import serial
import time

# reading data from arduino
def write_read():
	time.sleep(.4)
	data = arduino.readline()

	return data

# main function
def callback():
	# initializing publishers
	gps_location_pub = rospy.Publisher('current_coordinate', array, queue_size=1)
	metal_detect_pub = rospy.Publisher('metal_detection', Int32, queue_size=1)
	reciever_pub = rospy.Publisher('reciever', array, queue_size=1)

	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		global gps_coor, dir_ang
		gps_coor = [0, 0]
		rec_info = [0,0]
		# dir_ang = 0
		connections = gps_location_pub.get_num_connections()
		connections1 = metal_detect_pub.get_num_connections()
		#connections2 = reciever_pub.get_num_connections()
		if connections > 0 and connections1 > 0:  # ensure there is connection before continuing
			value = write_read()
			values = value.decode('utf-8')
			if len(values) > 24:  # ensure there are enough values to publish
				lat = float(values[0:8])
				lng = float(values[11:20])
				metal_detect = int(values[22])
				rec1 = int(values[24])
				rec2 = int(values[26])

				gps_coor[0] = lat
				gps_coor[1] = lng

				rec_info[0] = rec1
				rec_info[1] = rec2

			if gps_coor[0] != 0:  # ensure we dont have a BS value for the gps coordinates
				gps_location_pub.publish(gps_coor) # publishing current location
				# rospy.loginfo("Current location:%s", gps_coor)

				metal_detect_pub.publish(metal_detect) # publishing metal detection
				# rospy.loginfo("Metal Detect: %s", metal_detect)

				reciever_pub.publish(rec_info) # publishing reciever info
				# rospy.loginfo("Reciever: %s", rec_info)

			#break
		else:
			rate.sleep()  # if no connection, sleep and continue loop

def recieve():
	rospy.init_node('microcontroller_info_pub')
	callback()
	rospy.spin()

if __name__ == "__main__": 
	arduino = serial.Serial(port='/dev/ttyACM0', baudrate=9600, timeout=.1)
	recieve()
