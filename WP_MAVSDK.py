#!/usr/bin/env python3

import asyncio
from mavsdk import System
from mavsdk import mission_raw
import time
from pymavlink import mavutil
from pymavlink import mavwp
import numpy as np


print("Connection initiated....\n")
the_connection = mavutil.mavlink_connection("udp:127.0.0.1:14550")
the_connection.wait_heartbeat()
print(
    "Heartbeat from system (system %u component %u)"
    % (the_connection.target_system, the_connection.target_component)
)

wp_list = []


async def px4_connect_drone():
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("-- Connected to drone!")
            return drone


async def run():
    drone = await px4_connect_drone()
    await write_pixhawk(drone)

'''
async def run_drone(drone):

    mission_items = []

    mission_items.append(
        mission_raw.MissionItem(
            0,  # start seq at 0
            6,
            16,
            1,  # first one is current
            1,
            0,
            10,
            0,
            float("nan"),
            int(47.40271757 * 10**7),
            int(8.54285027 * 10**7),
            30.0,
            0,
        )
    )

    mission_items.append(
        mission_raw.MissionItem(
            1,
            6,
            16,
            0,
            1,
            0,
            10,
            0,
            float("nan"),
            int(47.40271757 * 10**7),
            int(8.54361892 * 10**7),
            30.0,
            0,
        )
    )

    print("-- Uploading mission")
    await drone.mission_raw.upload_mission(wp_list)
    print("-- Done")
'''




async def arm():
    the_connection.mav.command_long_send(
        the_connection.target_system,
        the_connection.target_component,
        400,
        1,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
    )
    is_arm = the_connection.recv_match(type="COMMAND_ACK", blocking=True)
    print(is_arm)
    time.sleep(2)
    print("DRONE IS ARMED")


async def disarm():
    the_connection.mav.command_long_send(
        the_connection.target_system,
        the_connection.target_component,
        400,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
    )
    is_disarm = the_connection.recv_match(type="COMMAND_ACK", blocking=True)
    print(is_disarm)
    time.sleep(2)
    print("DRONE IS DISARMED")


async def write_pixhawk(drone):
    # Enter the WayPoint file path
    with open("/home/nerdnhk/controlsystems/college.csv") as f:
        next(f)  # Skip the first line
        next(f)  # Skip the second line
        for line in f:
            fields = line.strip().split(",")

            # line_num = 0
            # for line in f:                   ******* USE THIS CODE IF ABOVE DOESNT WORK *******
            # if line_num == 0:
            # line_num += 1
            # continue  # Skip the first line

            # Defining each field in the file
            seq = int(fields[0])
            current = int(round(float(fields[1])))
            frame = int(round(float(fields[2])))
            command = int(round(float(fields[3])))
            param_1 = int(round(float(fields[4])))
            param_2 = int(round(float(fields[5])))
            param_3 = int(round(float(fields[6])))
            param_4 = int(round(float(fields[7])))
            latitude = float(fields[8])
            longitude = float(fields[9])
            altitude = float(fields[10])
            print(
                f"Waypoint: ({latitude}, {longitude}, {altitude})"
            )  # Just for debugging, can be removed right after
            # Writing the parameters into the pixhawk
            wp_list.append(
                mission_raw.MissionItem(
                    seq,
                    frame,
                    command,
                    current,
                    0,  # autocontinue
                    param_1,
                    param_2,
                    param_3,
                    param_4,
                    np.int32(10**7 * latitude),
                    np.int32(10**7 * longitude),
                    altitude,
                    16
                )
            )
            # the_connection.send(wp_list)
            print(wp_list)
            print("Waypoint is written into the pixhawk")
    print("-- Uploading mission")
    await drone.mission_raw.upload_mission(wp_list)
    print("-- Done")
    # print("ALL WAYPOINTS WRITTEN INTO PIXHAWK")


async def auto_mode():
    await the_connection.mav.command_long_send(
        the_connection.target_system,
        the_connection.target_component,
        176,
        0,
        1,
        3,
        0,
        0,
        0,
        0,
        0,
    )
    auto = the_connection.recv_match(type="COMMAND_ACK", blocking=True)
    print(auto)
    print("DRONE SET TO AUTO MODE")

async def mission_start():
    # to Start the mission
    await the_connection.mav.command_long_send(
        the_connection.target_system,
        the_connection.target_component,
        300,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0)
    msn_str = the_connection.recv_match(type="COMMAND_ACK", blocking=True)
    print(msn_str)
    print("MISSION STARTING")

# to start a connection listening on a UDP port
the_connection = mavutil.mavlink_connection("udp:127.0.0.1:14550")

# Waiting for first heartbeat message
the_connection.wait_heartbeat()
print(
    "Heartbeat from system (system %u component %u)"
    % (the_connection.target_system, the_connection.target_component)
)

# Connecting to the autopilot
# master = mavutil.mavlink_connection('udpout:localhost:14550')

# Connecting to Pixhawk
# master = mavutil.mavlink_connection('/dev/ttyACM0', baud=57600)
"""
write_pixhawk()
arm()
auto_mode()
mission_start()
"""
if __name__ == "__main__":
    # Run the asyncio loop
    asyncio.run(run())
    asyncio.run(write_pixhawk(px4_connect_drone))
    asyncio.run(auto_mode())

# Waiting for the mission to end
msg = the_connection.recv_match(type="MISSION_ITEM_REACHED", blocking=True)
print(msg)
# Ending connection
the_connection.close()
