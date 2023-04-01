import os
import time
import pandas as pd
from math import sqrt
import numpy as np
from pymavlink import mavutil
from pymavlink import mavwp

file_dis_list = []
curr_lat = np.float32(0)
curr_lon = np.float32(0)

waypoints_dir = '/home/nerdnhk/pymavlink/wpf_folder/'

# set the directory where the output .csv files will be saved
csv_dir = '/home/nerdnhk/pymavlink/csv_files/'

file_dis_list = []
wpf_final = ""
the_connection = mavutil.mavlink_connection('udpin:localhost:14551')
def init_conn():

    # Wait for the first heartbeat 
    #   This sets the system and component ID of remote system for the link
    the_connection.wait_heartbeat()
    print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))
def get_home_gps():
    while True:
        isArm = the_connection.recv_match(type='HOME_POSITION', blocking=True)
        # Check if the system is armed
        print("Waitng to ARM the drone\n")
        mode = mavutil.mode_mapping('LOITER')
        custom_mode = 4
        the_connection.set_mode(mode, custom_mode)
        print('DRONE IN LOITER MODE')
        if isArm.system_status == mavutil.mavlink.MAV_STATE_ACTIVE:
            print("System is armed.")
            msg = the_connection.recv_match(type="GLOBAL_POSITION_INT",blocking = True)
            print(msg)
            curr_lat = (msg.lat)/1e7
            curr_lon = (msg.lon)/1e7
            break
        time.sleep(0.25)
        for filename in os.listdir(waypoints_dir):
            if filename.endswith('.waypoints'):
                #uncomment below once to create a new directory, then uncomment it
                #os.mkdir(csv_dir)
                #time.sleep(2)
                
                # read the file into a pandas dataframe
                df = pd.read_csv(os.path.join(waypoints_dir, filename), delimiter='\t', header=None, names=['waypoint_number', 'ishome','confirmation', 'dont_know_1','param_1', 'param_2', 'param_3', 'param_4', 'latitude', 'longitude', 'altitude','dont_know_2'])

                # select specific columns from the dataframe
                selected_columns = df[['waypoint_number', 'ishome','confirmation', 'dont_know_1','param_1', 'param_2', 'param_3', 'param_4', 'latitude', 'longitude', 'altitude','dont_know_2']]

                # construct the output filename
                output_filename = os.path.splitext(filename)[0] + '.csv'

                # write the selected columns to a new .csv file
                selected_columns.to_csv(os.path.join(csv_dir, output_filename), index=False)
def ext_gps(csv_filename):
    df = pd.read_csv(os.path.join(csv_dir,csv_filename), usecols=['dont_know_1', 'latitude','longitude'])
    #print(df['dont_know_1'])
    f_lat = np.float32(0)
    f_lon = np.float32(0)
    #print(f_lat)
    for i in range(1,3):
        row = df.loc[i, :]
        #print(row['dont_know_1'])
        if row['dont_know_1'] !=22 and row.loc['latitude'] != 0.0 and row.loc['latitude'] !=0.0:
            #print(row)
            f_lat = row.loc['latitude']
            print(f_lat)
            f_lon = row.loc['longitude']
            print(f_lon)
            break
    dis = sqrt((f_lat-curr_lat)**2+(f_lon-curr_lon)**2)
    #print(dis)
    file_dis_list.append(dis)
file_name_list = os.listdir(csv_dir)
for filename in os.listdir(waypoints_dir):
        if filename.endswith('.waypoints'):
            #uncomment below once to create a new directory, then uncomment it
            #os.mkdir(csv_dir)
            #time.sleep(2)
            
            # read the file into a pandas dataframe
            df = pd.read_csv(os.path.join(waypoints_dir, filename), delimiter='\t', header=None, names=['waypoint_number', 'ishome','confirmation', 'dont_know_1','param_1', 'param_2', 'param_3', 'param_4', 'latitude', 'longitude', 'altitude','dont_know_2'])

            # select specific columns from the dataframe
            selected_columns = df[['waypoint_number', 'ishome','confirmation', 'dont_know_1','param_1', 'param_2', 'param_3', 'param_4', 'latitude', 'longitude', 'altitude','dont_know_2']]

            # construct the output filename
            output_filename = os.path.splitext(filename)[0] + '.csv'

            # write the selected columns to a new .csv file
            selected_columns.to_csv(os.path.join(csv_dir, output_filename), index=False)
print(file_name_list)
for file_name in file_name_list:
    #print(file_name)
    csv_file_path = os.path.join(csv_dir,file_name)
    print("Calculating minium distance\n")
    if os.path.isfile(csv_file_path):
        print(file_name+" CSV file found")
        ext_gps(csv_file_path)
    else:
        print("CSV file not found")
#ext_gps('college.csv')
min_dis_wpf = min(file_dis_list)
pd_series = pd.Series(file_dis_list)
min_dis_index = pd_series.index[pd_series == min_dis_wpf][0]
print("The shortest distance wp file:"+str(min_dis_index))
#print(pd_series)
wpf_final = file_name_list[min_dis_index]
print("Final WP file seleted: "+wpf_final)
#print(file_dis_list)

                             
                             
                             

        
