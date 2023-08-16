#!/usr/bin/env python3
import os
import time
import asyncio
import numpy as np
import pandas as pd
from pymavlink import mavwp
from pymavlink import mavutil
from math import cos,sin,atan2,radians,sqrt

# set the directory where the .waypoints files are located
#waypoints_dir = '/home/raspi/Waypoint-auto-mission/waypoint/' #for raspi
waypoints_dir = '/home/nerdnhk/upload_mission/wpf_folder/'

# set the directory where the output .csv files will be saved
#csv_dir = '/home/raspi/Waypoint-auto-mission/csv/' #for raspi
csv_dir = '/home/nerdnhk/upload_mission/csv_files/'


# fill MAVLINK ID one use '/dev/ttyusbx' (x is a int), 'UDP' or 'TCP' #udpin://:14550 
MAVLINK_ID1 = "udpin:localhost:14550"
baud1 = 57600

# Fill the required range, default 100m
wp_range = 100 

# Start a connection listening on a UDP port
print("Trying to establish a MavLink connection with the Drone...")
the_connection = mavutil.mavlink_connection(MAVLINK_ID1)

# Wait for the first heartbeat 
print("Waiting For HeartBeat")
the_connection.wait_heartbeat()

# This sets the system and component ID of remote system for the link
print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))

print("Loading Waypoint Loader")
wp = mavwp.MAVWPLoader() 

no_tries = 0
wp_list = []
wp_file_list = []
file_dis_list = []
csv_file_list = []
curr_lat = np.float32(0)
curr_lon = np.float32(0)
alt = np.float32(0)
wpf_final = ""

def wpToCSV(waypoints_dir):
    print("Converting waypoints file to CSV")
    for filename in os.listdir(waypoints_dir):
        #print("Converting "+str(filename)+" to CSV")
        if filename.endswith('.waypoints'):
            # read the file into a pandas dataframe
            df = pd.read_csv(os.path.join(waypoints_dir, filename), delimiter='\t', header=None, names=['seq', 'current','frame', 'command','param_1', 'param_2', 'param_3', 'param_4', 'latitude', 'longitude', 'altitude','mission_type'])

            # construct the output filename
            output_filename = os.path.splitext(filename)[0] + '.csv'

            # write the selected columns to a new .csv file
            df.to_csv(os.path.join(csv_dir, output_filename), index=False)

def ext_gps(csv_file_path,gps):
    # Extract 3 columns from CSV file
    df = pd.read_csv(csv_file_path, usecols=['command', 'latitude','longitude'])
    f_lat = np.float32(0)
    f_lon = np.float32(0)
    
    for i in range(1,3):
        row = df.loc[i, :]
        if row['command'] !=22 and row.loc['latitude'] != 0.0 and row.loc['latitude'] !=0.0:
            f_lat = row.loc['latitude']
            f_lon = row.loc['longitude']
            break
    dis = distance_lat_lon(f_lat,f_lon,np.float32(gps[0]),np.float32(gps[1]))
    print("Found distance is "+str(dis/1000)+"km")
    file_dis_list.append(dis)

def uploadmission(aFileName,gps):
    #home_location = [curr_lat,curr_lon]
    home_altitude = None
    total_lines = 0
    with open(aFileName) as f2:
        for i, line in enumerate(f2):
            total_lines = i-2
        print("Total number of lines: "+str(total_lines))

    with open(aFileName) as f:
        next(f)
        for i, line in enumerate(f): #i iterates as normal int and line iterates every line in the file
            if i == 0:
                print("Started Waypoint Parsing")
            else:
                #split all the words in the line
                linearray = line.strip().split(',')
                #assigning every param to a variable
                ln_seq = int(linearray[0])
                ln_current = int(round(float(linearray[1])))
                ln_frame = int(round(float(linearray[2])))
                ln_command = int(round(float(linearray[3])))
                ln_param1 = float(linearray[4])
                ln_param2 = float(linearray[5])
                ln_param3 = float(linearray[6])
                ln_param4 = float(linearray[7])
                ln_x = int(float(linearray[8])*1e7)
                ln_y = int(float(linearray[9])*1e7)
                ln_z = float(linearray[10])
                ln_autocontinue = int(round(float(linearray[11])))
                if i == 2 and ln_command == mavutil.mavlink.MAV_CMD_NAV_WAYPOINT:
                    home_altitude = int(ln_z)
                    ln_command = mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
                    print("Changed first WayPoint to TakeOff ")
                if i == 1:
                    print("Changed Home Location to current Drone Location")
                    ln_x = int(round(float(gps[0]))*1e7)
                    ln_y = int(round(float(gps[1]))*1e7)
                '''
                if ln_seq == 11 and ln_command == mavutil.mavlink.MAV_CMD_NAV_WAYPOINT:
                    home_altitude = int(ln_z)
                    ln_command = mavutil.mavlink.MAV_CMD_NAV_LAND
                    print("Changed last WayPoint to LAND ")
                '''        
                print(f"Waypoint: ({ln_command}, {ln_x/1e7}, {ln_y/1e7}, {ln_z})")
                p = mavutil.mavlink.MAVLink_mission_item_int_message(the_connection.target_system, the_connection.target_component, ln_seq,
                                                                 ln_frame,
                                                                 ln_command,
                                                                 ln_current, ln_autocontinue, ln_param1, ln_param2,
                                                                 ln_param3, ln_param4, ln_x, ln_y, ln_z, 0)
                wp.add(p)
    # send waypoint to airframe
    the_connection.waypoint_clear_all_send()
    the_connection.waypoint_count_send(wp.count())
    print()
