#!/usr/bin/env python3

from tkinter import *

def inverse():
	pass

def new_loca():
    lat_waypoint = float(str_way_lat.get())
    long_waypoint = float(str_way_long.get())
    # lat_home = float(str_home_lat.get())
    # long_home = float(str_home_long.get())
    print(lat_waypoint)
    print(long_waypoint)

def minelogger():
    mineLog.insert(0.0, "Mine location here\n")

root = Tk() #Makes the window
root.wm_title("Control Center") #Makes the title that will appear in the top left
root.config(background = "white")
root.configure(background="black")

str_way_lat = StringVar() 
str_way_long = StringVar() 
str_home_lat = StringVar() 
str_home_long = StringVar() 

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

mineLog = Text(leftFrame1, width = 30, height = 10, takefocus=0)
mineLog.grid(row=1, column=0, padx=10, pady=2)

current_location_lat = Label(rightFrame, text='25.7149331', width=15)
current_location_lat.grid(row=0,column=0, padx=10, pady=2)
current_location_lat.config(font =("Calibre", 10))

current_location_long = Label(rightFrame, text='-80.361461', width=15)
current_location_long.grid(row=0,column=1, padx=10, pady=2)
current_location_long.config(font =("Calibre", 10))

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
auto_button = Button(rightFrame1, text='AutoPilot', fg='black', bg='light green', command=lambda:inverse, height=1, width=7) 
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