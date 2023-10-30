from dronekit import connect, VehicleMode
import time 
drone = connect('127.0.0.1:14550')
drone.initialize()
 
# drone.arm()

mode = drone.mode.name
print(drone.send_mavlink)
print(f"Vehicle is in {mode} mode.")

print('Changing to guided ...')
drone.mode = VehicleMode('LAND')
time.sleep(3)

mode = drone.mode.name
print(drone.send_mavlink)
print(f"Vehicle is in {mode} mode.")

# drone.armed = True
# # Change to loiter

# target_altitude = 10  # Change to desired altitude in meters
# drone.simple_takeoff(target_altitude)
# drone.wait_simple_takeoff(10)
while True:
    print(drone.location.local_frame.distance_home())
    time.sleep(1)
