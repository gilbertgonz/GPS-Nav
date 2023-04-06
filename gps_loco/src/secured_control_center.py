#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32, Float32, Bool, Float32MultiArray, String
from sensor_msgs.msg import NavSatFix
import customtkinter
from tkinter import *

security_check = False

class SecurityWall:
    def __init__(self, password):
        self.password = password
        self.root = Tk()
        self.root.title('Security Wall')
        self.root.geometry('300x100')
        self.password_label = Label(self.root, text='Enter Password:')
        string = "Enter Password"
        status_string_pub.publish(string)
        print(string)
        self.password_label.pack()
        self.password_entry = Entry(self.root, show='*')
        self.password_entry.pack()
        self.submit_button = Button(self.root, text='Submit', command=self.check_password)
        self.submit_button.pack()
        self.status_label = Label(self.root, text='')
        self.status_label.pack()
        
    def check_password(self):
        global security_check
        if self.password_entry.get() == self.password:
            self.status_label.config(text='Access granted.')
            rospy.loginfo("Password is correct")
            self.root.destroy()
            security_check = True
        else:
            self.status_label.config(text='Incorrect password. Please try again.')

    def run(self):
        self.root.mainloop()

def new_loca():
    try:
        lat_waypoint = float(str_way_lat.get())
        long_waypoint = float(str_way_long.get())
        # lat_home = float(str_home_lat.get())
        # long_home = float(str_home_long.get())

        waypoint = NavSatFix()
        waypoint.latitude = lat_waypoint
        waypoint.longitude = long_waypoint

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
        string = "Enter proper coord"
        status_string_pub.publish(string)
        print(string)

def stoppilot():
    global auto
    auto = False
    autopilot_pub.publish(auto) # publishing stoppilot state variable

def autopilot():
    auto = True
    autopilot_pub.publish(auto) # publishing autopilot state variable

def ret_home():
    try:
        lat_home = float(str_home_lat.get())
        long_home = float(str_home_long.get())

        home_waypoint = NavSatFix()
        home_waypoint.latitude = lat_home
        home_waypoint.longitude = long_home

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            connections = gps_waypoint_pub.get_num_connections()

            if connections > 0:  # ensuring there is a connection before publishing
                gps_waypoint_pub.publish(home_waypoint) # publishing home coordinate
                # rospy.loginfo("Home location: %s", waypoint)
                break
            else:
                rate.sleep()  # if no connection, sleep and continue loop
                
    
    except ValueError:
        string = "Enter home coord"
        status_string_pub.publish(string)
        print(string)

def img_capture():
    global capture
    capture = 1
    img_cap_pub.publish(capture) # publish image capture state variable

def minelogger():
    mineLog.insert(0.0, "(metal)")
    mineLog.insert(0.0, " ")
    mineLog.insert(0.0, str(round(long1, 11)))
    mineLog.insert(0.0, ", ")
    mineLog.insert(0.0, str(round(lat1, 11)))
    mineLog.insert(0.0, "\n")

    connections = mine_coor_pub.get_num_connections()
    mine_coor = NavSatFix()
    mine_coor.latitude = lat1
    mine_coor.longitude = long1

    if connections > 0:  # ensuring there is a connection before publishing
        mine_coor_pub.publish(mine_coor) # publishing mine coordinate

def mineloggerCV():
    mineLog.insert(0.0, "(" + mine_type + ")")
    mineLog.insert(0.0, " ")
    mineLog.insert(0.0, str(round(long1, 11)))
    mineLog.insert(0.0, ", ")
    mineLog.insert(0.0, str(round(lat1, 11)))
    mineLog.insert(0.0, "\n")

    connections = mine_coor_pub.get_num_connections()
    mine_coor = NavSatFix()
    mine_coor.latitude = lat1
    mine_coor.longitude = long1

    if connections > 0:  # ensuring there is a connection before publishing
        mine_coor_pub.publish(mine_coor) # publishing mine coordinate

def lat_current1():
    if lat_current.winfo_exists():
        lat_current.delete(0, 'end')
        lat_current.insert(0, str(round(lat1, 13)))

def long_current1():
    if long_current.winfo_exists():
        long_current.delete(0, 'end')
        long_current.insert(0, str(round(long1, 13)))

def metal_detection_callback(data):  
    global metal_detect

    metal_detect = data.data
    # print(metal_detect)
    if metal_detect > 0:
        string = "Metal detected"
        mine_string_pub.publish(string)
        minelogger()  # if metal is detected, call the mineLogger function to log the current location of the detected mine
    else:
        mine_string_pub.publish("")
        
    capture = 0
    img_cap_pub.publish(capture) # publishing image capture state variable as zero to avoid conflicts with publisher
     
def current_location_callback(data):
    global lat1, long1
    lat1 = data.latitude
    long1 = data.longitude
    lat_current1()
    long_current1()

def confidenceCV_callback(data):
    global conf
    conf = data.data
    
    if conf > 0.85:
        string = mine_type + " detected!"
        mine_string_pub.publish(string)
        mineloggerCV()
    else:
        mine_string_pub.publish("")

def classCV_callback(data):
    global mine_type
    cla = data.data

    if cla == 0:
        mine_type = "VS50"
    if cla == 1:
        mine_type = "PFM-1"
    if cla == 2:
        mine_type = "PMN"

def main():
    global str_way_lat, str_way_long, str_home_lat, str_home_long, lat_current, long_current, mineLog

    customtkinter.set_appearance_mode("Dark")  # Modes: "System" (standard), "Dark", "Light"

    root1 = Tk()
    root1.wm_title("Control Center")
    root1.configure(background="black")

    str_way_lat = StringVar() 
    str_way_long = StringVar() 
    str_home_lat = StringVar() 
    str_home_long = StringVar() 

    oneFrame =customtkinter.CTkFrame(root1, width=100, height = 480)
    oneFrame.grid(row=0, column=0, columnspan=2, padx=1, pady=5)

    leftFrame1 =customtkinter.CTkFrame(root1, width=200, height = 480)
    leftFrame1.grid(row=1, column=0, padx=5, pady=5)

    rightFrame1 =customtkinter.CTkFrame(root1, width=200, height = 100)
    rightFrame1.grid(row=1, column=1, padx=5, pady=5)

    rightFrame2 =customtkinter.CTkFrame(root1, width=200, height = 100)
    rightFrame2.grid(row=2, column=1, padx=5, pady=5)

    current_location_text = customtkinter.CTkLabel(oneFrame, text='Current Location:', anchor="e")
    current_location_text.grid(row=0,column=0, padx=10, pady=2)
    current_location_text.configure(font =("Roboto", 15))

    waypoint_location_text = customtkinter.CTkLabel(oneFrame, text='Waypoint Location:', anchor="e")
    waypoint_location_text.grid(row=1,column=0, padx=10, pady=2)
    waypoint_location_text.configure(font =("Roboto", 15))

    home_location_text = customtkinter.CTkLabel(oneFrame, text='Home Location:', anchor="e")
    home_location_text.grid(row=2,column=0, padx=10, pady=2)
    home_location_text.configure(font =("Roboto", 15))

    detected_mines_text = customtkinter.CTkLabel(leftFrame1, text='Detected Mines:',anchor="e")
    detected_mines_text.grid(row=0,column=0, padx=10, pady=2)
    detected_mines_text.configure(font =("Roboto", 15))

    mineLog = customtkinter.CTkTextbox(leftFrame1, width=350)
    mineLog.grid(row=1, column=0, padx=5, pady=2)

    lat_current = customtkinter.CTkEntry(oneFrame)
    lat_current.grid(row=0, column=1, padx=10, pady=2)
    lat_current.configure(font =("Roboto", 14))
    
    long_current = customtkinter.CTkEntry(oneFrame)
    long_current.grid(row=0, column=2, padx=10, pady=2)
    long_current.configure(font =("Roboto", 14))
    
    waypoint_lat = customtkinter.CTkEntry(oneFrame, textvariable = str_way_lat)
    waypoint_lat.grid(row=1, column=1, padx=10, pady=2)
    waypoint_lat.configure(font =("Roboto", 14))

    waypoint_long = customtkinter.CTkEntry(oneFrame, textvariable = str_way_long)
    waypoint_long.grid(row=1, column=2, padx=10, pady=2)
    waypoint_long.configure(font =("Roboto", 14))

    home_lat = customtkinter.CTkEntry(oneFrame, textvariable = str_home_lat)
    home_lat.grid(row=2, column=1, padx=10, pady=2)
    home_lat.configure(font =("Roboto", 14))

    home_long = customtkinter.CTkEntry(oneFrame, textvariable = str_home_long)
    home_long.grid(row=2, column=2, padx=10, pady=2)
    home_long.configure(font =("Roboto", 14))

    battery_life_text = customtkinter.CTkLabel(rightFrame2, text='Battery Life:', corner_radius=10)
    battery_life_text.grid(row=0, column=0)
    battery_life_text.configure(font =("Roboto", 15))

    battery_number = customtkinter.CTkLabel(rightFrame2, text='80%', corner_radius=10)
    battery_number.grid(row=0, column=1)
    battery_number.configure(font =("Roboto", 15))

    # creating buttons
    auto_button =customtkinter.CTkButton(rightFrame1, text='AutoPilot', corner_radius=10, fg_color="light green", text_color="black", command=autopilot, font=("Roboto", 15)) 
    auto_button.grid(row=0, column=0, padx=5, pady=5) 

    return_button =customtkinter.CTkButton(rightFrame1, text='Return Home', corner_radius=10, fg_color="light yellow", text_color="black", command=ret_home, font=("Roboto", 15)) 
    return_button.grid(row=1, column=0, padx=15, pady=5)

    new_loca_button =customtkinter.CTkButton(rightFrame1, text='New Waypoint', corner_radius=10, fg_color="white", text_color="black", command=new_loca, font=("Roboto", 15)) 
    new_loca_button.grid(row=0, column=1, padx=5, pady=5)

    cap_img =customtkinter.CTkButton(rightFrame1, text='Capture Image', corner_radius=10, fg_color="light blue", text_color="black", command=img_capture, font=("Roboto", 15)) 
    cap_img.grid(row=1, column=1, padx=5, pady=5)

    stop_button =customtkinter.CTkButton(rightFrame1, text='Stop (RC)', corner_radius=10, fg_color="red", text_color="white", command=stoppilot, font=("Roboto", 15)) 
    stop_button.grid(row=2, column=0, columnspan=2, padx=10, pady=10)

    root1.mainloop()

def recieve():
    rospy.Subscriber('metal_detection', Bool, metal_detection_callback, queue_size=1)
    rospy.Subscriber('current_coordinate', NavSatFix, current_location_callback, queue_size=1)
    rospy.Subscriber('target_conf', Float32, confidenceCV_callback, queue_size=1)
    rospy.Subscriber('target_class', Int32, classCV_callback, queue_size=1)
    main()
    rospy.spin()

if __name__ == "__main__": 
    rospy.init_node('gui_control_new')
    password = '1'
    status_string_pub = rospy.Publisher('status_string', String, queue_size=1)
    security_wall = SecurityWall(password)
    security_wall.run()

    if security_check:
        gps_waypoint_pub = rospy.Publisher('waypoint_coordinate', NavSatFix, queue_size=1)
        mine_coor_pub = rospy.Publisher('mine_coordinate', NavSatFix, queue_size=1)
        autopilot_pub = rospy.Publisher('autopilot_check', Bool, queue_size=1)
        img_cap_pub = rospy.Publisher('img_capture', Int32, queue_size=1)
        mine_string_pub = rospy.Publisher('mine_string', String, queue_size=1)
        recieve()
    