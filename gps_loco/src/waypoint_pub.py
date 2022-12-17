#!/usr/bin/env python3

import rospy
from array_msgs.msg import array
from std_msgs.msg import Int32
from tkinter import *

def new_loca():
    try:
        lat_waypoint = float(str_way_lat.get())
        long_waypoint = float(str_way_long.get())
        # lat_home = float(str_home_lat.get())
        # long_home = float(str_home_long.get())

        waypoint = [0, 0]
        waypoint[0] = lat_waypoint
        waypoint[1] = long_waypoint

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            connections = gps_waypoint_pub.get_num_connections()

            if connections > 0:  # ensuring there is a connection before publishing
                gps_waypoint_pub.publish(waypoint) # publishing waypoint coordinate
                # rospy.loginfo("Waypoint location: %s", waypoint)
                break
            else:
                rate.sleep()  # if no connection, sleep and continue loop
                
    
    except ValueError:
        print("Enter a proper coordinate")

def minelogger():
    mineLog.insert(0.0, str(round(long1, 8)))
    mineLog.insert(0.0, ", ")
    mineLog.insert(0.0, str(round(lat1, 8)))
    mineLog.insert(0.0, "\n")

def lat_current1():
    lat_current.insert(0.0, str(round(lat1, 8)))

def long_current1():
    long_current.insert(0.0, str(round(long1, 8)))

def callback1(data):  
    global metal_detect

    metal_detect = data.data
    # print(metal_detect)
    if metal_detect > 0:
        minelogger()  # if metal is detected, call the mineLogger function to log the current location of the detected mine
        
def callback2(data):
    global lat1, long1
    cur_loca = data.data
    rospy.loginfo("GPS: %s", data.data)
    lat1, long1 = cur_loca[0], cur_loca[1]
    lat_current1()
    long_current1()

def callback():
    global str_way_lat, str_way_long, str_home_lat, str_home_long, lat_current, long_current

    root = Tk()
    root.wm_title("Control Center")
    root.config(background="black")

    str_way_lat = StringVar() 
    str_way_long = StringVar() 
    str_home_lat = StringVar() 
    str_home_long = StringVar() 

    def inverse():
        pass
    
    leftFrame = Frame(root, width=200, height = 480)
    leftFrame.grid(row=0, column=0, padx=1, pady=5)

    leftFrame1 = Frame(root, width=200, height = 480)
    leftFrame1.grid(row=1, column=0, padx=5, pady=5)

    rightFrame = Frame(root, width=440, height = 100)
    rightFrame.grid(row=0, column=1, padx=5, pady=5)

    rightFrame1 = Frame(root, width=200, height = 100)
    rightFrame1.grid(row=1, column=1, padx=5, pady=5)

    rightFrame2 = Frame(root, width=200, height = 100)
    rightFrame2.grid(row=2, column=1, padx=5, pady=5)

    current_location_text = Label(leftFrame, text='Current Location:', width=16)
    current_location_text.grid(row=0,column=0, padx=10, pady=2)
    current_location_text.config(font =("Calibre", 12))

    waypoint_location_text = Label(leftFrame, text='Waypoint Location:', width=16)
    waypoint_location_text.grid(row=1,column=0, padx=10, pady=2)
    waypoint_location_text.config(font =("Calibre", 12))

    home_location_text = Label(leftFrame, text='Home Location:', width=16)
    home_location_text.grid(row=2,column=0, padx=10, pady=2)
    home_location_text.config(font =("Calibre", 12))

    detected_mines_text = Label(leftFrame1, text='Detected Mines:', width=16)
    detected_mines_text.grid(row=0,column=0, padx=10, pady=2)
    detected_mines_text.config(font =("Calibre", 12))

    global mineLog
    mineLog = Text(leftFrame1, width = 30, height = 10, takefocus=0)
    mineLog.grid(row=1, column=0, padx=10, pady=2)

    lat_current = Text(rightFrame, width = 12, height = 1, bg='light yellow')
    lat_current.grid(row=0, column=0, padx=10, pady=2)
    
    long_current = Text(rightFrame, width = 12, height = 1, bg='light yellow')
    long_current.grid(row=0, column=1, padx=10, pady=2)
    
    waypoint_lat = Entry(rightFrame, textvariable = str_way_lat, width=12,bg='light yellow')
    waypoint_lat.grid(row=1, column=0, padx=10, pady=2)

    waypoint_long = Entry(rightFrame, textvariable = str_way_long, width=12,bg='light yellow')
    waypoint_long.grid(row=1, column=1, padx=10, pady=2)

    home_lat = Entry(rightFrame, textvariable = str_home_lat, width=12,bg='light yellow')
    home_lat.grid(row=2, column=0, padx=10, pady=2)

    home_long = Entry(rightFrame, textvariable = str_home_long, width=12,bg='light yellow')
    home_long.grid(row=2, column=1, padx=10, pady=2)

    battery_life_text = Label(rightFrame2, text='Battery Life:', width=12)
    battery_life_text.grid(row=0, column=0)
    battery_life_text.config(font =("Calibre", 12))

    battery_number = Label(rightFrame2, text='80%', width=5)
    battery_number.grid(row=0, column=1)
    battery_number.config(font =("Calibre", 12))

    # creating buttons
    auto_button = Button(rightFrame1, text='AutoPilot', fg='black', bg='light green', command=lat_current1, height=1, width=7) 
    auto_button.grid(row=0, column=0, padx=10, pady=10) 

    return_button = Button(rightFrame1, text='Return Home', fg='black', bg='light yellow', command=lambda:inverse, height=1, width=10) 
    return_button.grid(row=1, column=0, padx=10, pady=10)

    new_loca_button = Button(rightFrame1, text='New Waypoint', fg='black', bg='white', command=new_loca, height=1, width=10) 
    new_loca_button.grid(row=0, column=1, padx=10, pady=10)

    cap_img = Button(rightFrame1, text='Capture Image', fg='black', bg='light blue', command=lambda:inverse, height=1, width=10) 
    cap_img.grid(row=1, column=1, padx=10, pady=10)

    stop_button = Button(rightFrame1, text='Stop', fg='white', bg='red', command=lambda:inverse, height=1, width=7) 
    stop_button.grid(row=2, column=0, padx=10, pady=10)

    root.mainloop()

def recieve():
    rospy.init_node('gui_control')
    rospy.Subscriber('current_coordinate', array, callback2, queue_size=1)
    rospy.Subscriber('metal_detection', Int32, callback1, queue_size=1)
    callback()
    rospy.spin()

if __name__ == "__main__": 
    gps_waypoint_pub = rospy.Publisher('waypoint_coordinate', array, queue_size=1)
    recieve()
    