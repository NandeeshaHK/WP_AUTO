import time
import math
from pymavlink import mavutil

# Function to establish a connection to the Pixhawk and listen for a heartbeat
def connect_to_pixhawk():
    while True:
        try:
            # Try to auto-detect the serial port
            serial_ports = mavutil.auto_detect_serial_unix()
            serial_ports.reverse()
            # Iterate through the detected serial ports
            for i in range(len(serial_ports)):
                serial_port = serial_ports[i].device
                print(serial_port)
                try:
                    the_connection  = mavutil.mavlink_connection(serial_port)
                    while True:
                        msg = the_connection.wait_heartbeat()
                        print('HeartBeat from pixhawk, connection object returned')
                        return the_connection
                except:
                    time.sleep(0.5)
                    pass
        except Exception as e:
            print(f"Error: {e}")
            time.sleep(1)  # Wait for a moment before retrying

# Call the function to connect to the Pixhawk
gps_data=[]
mavlink_connection = connect_to_pixhawk()

#Example code to check the messages received, Position and Extra1 param has to be enabled.
while True:
    # For this type of collecting, make sure the refresh rate or the measure rate is set the same for all, recommended 2Hz
    msg = mavlink_connection.recv_match(type=['GLOBAL_POSITION_INT', 'ATTITUDE'], blocking=True)
    
    if msg.get_type() == 'GLOBAL_POSITION_INT':
        # Create a new GPS data entry
        gps_entry = (msg.lat / 1e7, msg.lon / 1e7, msg.relative_alt / 1e3, ((msg.hdg / 100)+0) % 360)

        # Acquire the lock to update the shared gps_data list
        gps_data.append(gps_entry)

    elif msg.get_type() == 'ATTITUDE':
        # Check if gps_data is not empty before updating
        if len(gps_data) > 0:
            if len(gps_data[-1]) == 4:
                yaw = math.degrees(msg.yaw)
                # Ensure yaw_deg is within the range [0, 360)
                if yaw < 0:
                    yaw += 360.0
                yaw = (yaw+0) % 360
                timestamp = time.time()
                # print(yaw)
            else:
                continue
            # print(gps_data[-1][:4] + (yaw, ))
            gps_data[-1] = gps_data[-1][:4] + (yaw, timestamp)
    if gps_data:
            try:
                parts = gps_data[-1]
                parts[5]
                print(gps_data[-1])
            except:
                pass
    time.sleep((1/(3*2)))
