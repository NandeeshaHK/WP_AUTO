#!/usr/bin/env python3
import os
import time
import asyncio
import numpy as np
import pandas as pd
from mavsdk import System
from mavsdk import mission_raw
from mavsdk.mission import (MissionItem, MissionPlan)
from pymavlink import mavwp
from pymavlink import mavutil
from math import cos,sin,atan2,radians,sqrt

# set the directory where the .waypoints files are located
waypoints_dir = '/home/nerdnhk/pymavlink/wpf_folder/'

# set the directory where the output .csv files will be saved
csv_dir = '/home/nerdnhk/pymavlink/csv_files/'

wp_list = []
wp_file_list = []
file_dis_list = []
csv_file_list = []
curr_lat = np.float32(0)
curr_lon = np.float32(0)
alt = np.float32(0)
wpf_final = ""


# Start a connection listening on a UDP port
the_connection = mavutil.mavlink_connection('udpin:localhost:14550')
wp = mavwp.MAVWPLoader()

async def init_conn():
    # Wait for the first heartbeat 
    print("Waiting For HeartBeat")
    the_connection.wait_heartbeat()
    # This sets the system and component ID of remote system for the link
    print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))

async def get_home_gps(drone):
    print("Waitng to ARM the drone\n")
    async for terrain_info in drone.telemetry.home():
        curr_lat = terrain_info.latitude_deg
        curr_lon = terrain_info.longitude_deg
        break
    print("Drone is ARMED for 5 secs")
    return (curr_lat,curr_lon)


def distance_lat_lon(lat1, lon1, lat2, lon2):
    '''distance between two points'''
    dLat = radians(lat2) - radians(lat1)
    dLon = radians(lon2) - radians(lon1)
    a = sin(0.5*dLat)**2 + sin(0.5*dLon)**2 * cos(lat1) * cos(lat2)
    c = 2.0 * atan2(sqrt(abs(a)), sqrt(abs(1.0-a)))
    ground_dist = 6371 * 1000 * c
    return ground_dist

def wpToCSV(waypoints_dir):
    for filename in os.listdir(waypoints_dir):
        if filename.endswith('.waypoints'):
            #uncomment below once to create a new directory, then uncomment it
            #os.mkdir(csv_dir)
            #time.sleep(2)
            
            # read the file into a pandas dataframe
            df = pd.read_csv(os.path.join(waypoints_dir, filename), delimiter='\t', header=None, names=['seq', 'current','frame', 'command','param_1', 'param_2', 'param_3', 'param_4', 'latitude', 'longitude', 'altitude','mission_type'])

            # construct the output filename
            output_filename = os.path.splitext(filename)[0] + '.csv'

            # write the selected columns to a new .csv file
            df.to_csv(os.path.join(csv_dir, output_filename), index=False)

def ext_gps(csv_file_path):
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
    dis = distance_lat_lon(f_lat,f_lon,curr_lat,curr_lon)
    print("Found distance is "+str(dis/1000)+"km")
    file_dis_list.append(dis)
    

# to find the shortest distance
def final_wpf():
    file_name_list = os.listdir(csv_dir)
    print(file_name_list)
    for file_name in file_name_list:
        csv_file_path = os.path.join(csv_dir,file_name)
        if os.path.isfile(csv_file_path):
            print(file_name+" CSV file found")
            (ext_gps(csv_file_path))
        else:
            print("CSV file not found")
    #to find minium distance WP file
    min_dis_wpf = min(file_dis_list)
    # convert Pandas Series for easy conversion
    pd_series = pd.Series(file_dis_list)
    min_dis_index = pd_series.index[pd_series == min_dis_wpf][0]
    print("The shortest distance wp file:"+str(min_dis_index))
    # now file _name_list consists only waypoints
    file_name_list = os.listdir(csv_dir)
    wpf_final = file_name_list[min_dis_index]
    print("Final WP file seleted: "+wpf_final)

    # delete the directory
    file_name_list = os.listdir(csv_dir)
    file_name_list.remove(wpf_final)
    print("after deletion of csv files excluded req"+str(file_name_list))
    for file_name in file_name_list:
        csv_file_path = os.path.join(csv_dir,file_name)
        os.remove(csv_file_path)
    return wpf_final

#init_conn()
wpToCSV(waypoints_dir)
wpf_final = final_wpf()

async def run():
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            break

    print_mission_progress_task = asyncio.ensure_future(
        print_mission_progress(drone))

    running_tasks = [print_mission_progress_task]
    termination_task = asyncio.ensure_future(
        observe_is_in_air(drone, running_tasks))

    mission_items = []
    final_path = os.path.join(csv_dir, wpf_final)
    print("CSV file Path:"+str(final_path))
    final_path = "/home/nerdnhk/controlsystems/college.csv"
    speed = int(input("Input Desired Drone Speed(m/s):"))
    with open(final_path) as f:
        next(f)  # Skip the first line
        next(f)  # Skip the second line
        for i,line in enumerate(f):
            fields = line.strip().split(",")
            seq = int(fields[0])
            latitude = float(fields[8])
            longitude = float(fields[9])
            altitude = float(fields[10])
            print(f"Waypoint: ({seq}, {latitude}, {longitude}, {altitude})")
            mission_items.append(MissionItem(latitude,
                                     longitude,
                                     altitude,
                                     speed,
                                     True,
                                     float('nan'),
                                     float('nan'),
                                     MissionItem.CameraAction.NONE,
                                     float('nan'),
                                     float('nan'),
                                     float('nan'),
                                     float('nan'),
                                     float('nan')))
    
    #async for flight_mode in drone.telemetry.flight_mode():
        #print("FlightMode:", flight_mode)    
    #await print_flight_mode(drone)
    mission_plan = MissionPlan(mission_items)

    await drone.mission.set_return_to_launch_after_mission(True)

    print("-- Uploading mission")
    await drone.mission.upload_mission(mission_plan)

    print("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position estimate OK")
            break

    print("-- Arming")
    await drone.action.arm()

    await asyncio.sleep(2)
    print("-- Starting mission")
    await drone.mission.start_mission()

    await termination_task

async def print_flight_mode(drone):
    """ Prints the flight mode when it changes """

    previous_flight_mode = None

    async for flight_mode in drone.telemetry.flight_mode():
        if flight_mode != previous_flight_mode:
            previous_flight_mode = flight_mode
            print(f"Flight mode: {flight_mode}")

async def print_mission_progress(drone):
    async for mission_progress in drone.mission.mission_progress():
        print(f"Mission progress: "
              f"{mission_progress.current}/"
              f"{mission_progress.total}")


async def observe_is_in_air(drone, running_tasks):
    """ Monitors whether the drone is flying or not and
    returns after landing """

    was_in_air = False

    async for is_in_air in drone.telemetry.in_air():
        if is_in_air:
            was_in_air = is_in_air

        if was_in_air and not is_in_air:
            for task in running_tasks:
                task.cancel()
                try:
                    await task
                except asyncio.CancelledError:
                    pass
            await asyncio.get_event_loop().shutdown_asyncgens()

            return


if __name__ == "__main__":
    # Run the asyncio loop
    asyncio.run(run())

# Ending connection
the_connection.close()

''' OUTPUT:
['random2.csv', 'college.csv', 'chikbalapur.csv', 'random1.csv', 'chikbalapur_spline.csv', 'search_grid_chikkballapur.csv']
random2.csv CSV file found
Found distance is 5636.692067050783km
college.csv CSV file found
Found distance is 8247.257938270468km
chikbalapur.csv CSV file found
Found distance is 6006.714477188903km
random1.csv CSV file found
Found distance is 8063.617013900678km
chikbalapur_spline.csv CSV file found
Found distance is 6006.714477188903km
search_grid_chikkballapur.csv CSV file found
Found distance is 6006.757180109768km
The shortest distance wp file:0
Final WP file seleted: random2.csv
after deletion of csv files excluded req['college.csv', 'chikbalapur.csv', 'random1.csv', 'chikbalapur_spline.csv', 'search_grid_chikkballapur.csv']
Waiting for drone to connect...
-- Connected to drone!
CSV file Path:/home/nerdnhk/pymavlink/csv_files/random2.csv
Input Desired Drone Speed(m/s):15
Waypoint: (0, 13.0311019, 77.5653581, 929.2)
Waypoint: (1, 13.0311036, 77.5653605, 15.24)
Waypoint: (2, 13.031174, 77.565558, 15.24)
Waypoint: (3, 13.0310355, 77.5656545, 15.24)
Waypoint: (4, 13.0308761, 77.565574, 15.24)
Waypoint: (5, 13.0308865, 77.5653662, 15.24)
Waypoint: (6, 13.0310315, 77.5652763, 15.24)
-- Uploading mission
Mission progress: 0/7
Waiting for drone to have a global position estimate...
-- Global position estimate OK
-- Arming
-- Starting mission
Traceback (most recent call last):
  File "/home/nerdnhk/controlsystems/exMAV_upload.py", line 239, in <module>
    asyncio.run(run())
  File "/home/nerdnhk/anaconda3/lib/python3.9/asyncio/runners.py", line 44, in run
    return loop.run_until_complete(main)
  File "/home/nerdnhk/anaconda3/lib/python3.9/asyncio/base_events.py", line 647, in run_until_complete
    return future.result()
  File "/home/nerdnhk/controlsystems/exMAV_upload.py", line 194, in run
    await drone.mission.start_mission()
  File "/home/nerdnhk/controlsystems/mavsdk/mission.py", line 1099, in start_mission
    raise MissionError(result, "start_mission()")
mavsdk.mission.MissionError: ERROR: 'Error'; origin: start_mission(); params: ()
exception calling callback for <Future at 0x7fbe185c3280 state=finished raised _MultiThreadedRendezvous>
Traceback (most recent call last):
  File "/home/nerdnhk/anaconda3/lib/python3.9/concurrent/futures/thread.py", line 58, in run
    result = self.fn(*self.args, **self.kwargs)
  File "/home/nerdnhk/anaconda3/lib/python3.9/site-packages/aiogrpc/utils.py", line 131, in _next
    return next(self._iterator)
  File "/home/nerdnhk/anaconda3/lib/python3.9/site-packages/grpc/_channel.py", line 426, in __next__
    return self._next()
  File "/home/nerdnhk/anaconda3/lib/python3.9/site-packages/grpc/_channel.py", line 826, in _next
    raise self
grpc._channel._MultiThreadedRendezvous: <_MultiThreadedRendezvous of RPC that terminated with:
	status = StatusCode.CANCELLED
	details = "Locally cancelled by application!"
	debug_error_string = "None"
>

During handling of the above exception, another exception occurred:

Traceback (most recent call last):
  File "/home/nerdnhk/anaconda3/lib/python3.9/concurrent/futures/_base.py", line 330, in _invoke_callbacks
    callback(self)
  File "/home/nerdnhk/anaconda3/lib/python3.9/asyncio/futures.py", line 398, in _call_set_state
    dest_loop.call_soon_threadsafe(_set_state, destination, source)
  File "/home/nerdnhk/anaconda3/lib/python3.9/asyncio/base_events.py", line 796, in call_soon_threadsafe
    self._check_closed()
  File "/home/nerdnhk/anaconda3/lib/python3.9/asyncio/base_events.py", line 515, in _check_closed
    raise RuntimeError('Event loop is closed')
RuntimeError: Event loop is closed
exception calling callback for <Future at 0x7fbe1458d7c0 state=finished raised _MultiThreadedRendezvous>
Traceback (most recent call last):
  File "/home/nerdnhk/anaconda3/lib/python3.9/concurrent/futures/thread.py", line 58, in run
    result = self.fn(*self.args, **self.kwargs)
  File "/home/nerdnhk/anaconda3/lib/python3.9/site-packages/aiogrpc/utils.py", line 131, in _next
    return next(self._iterator)
  File "/home/nerdnhk/anaconda3/lib/python3.9/site-packages/grpc/_channel.py", line 426, in __next__
    return self._next()
  File "/home/nerdnhk/anaconda3/lib/python3.9/site-packages/grpc/_channel.py", line 826, in _next
    raise self
grpc._channel._MultiThreadedRendezvous: <_MultiThreadedRendezvous of RPC that terminated with:
	status = StatusCode.CANCELLED
	details = "Locally cancelled by application!"
	debug_error_string = "None"
>

During handling of the above exception, another exception occurred:

Traceback (most recent call last):
  File "/home/nerdnhk/anaconda3/lib/python3.9/concurrent/futures/_base.py", line 330, in _invoke_callbacks
    callback(self)
  File "/home/nerdnhk/anaconda3/lib/python3.9/asyncio/futures.py", line 398, in _call_set_state
    dest_loop.call_soon_threadsafe(_set_state, destination, source)
  File "/home/nerdnhk/anaconda3/lib/python3.9/asyncio/base_events.py", line 796, in call_soon_threadsafe
    self._check_closed()
  File "/home/nerdnhk/anaconda3/lib/python3.9/asyncio/base_events.py", line 515, in _check_closed
    raise RuntimeError('Event loop is closed')
RuntimeError: Event loop is closed
'''
