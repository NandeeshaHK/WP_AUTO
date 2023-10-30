
from dronekit import connect, VehicleMode
import dronekit
import keyboard
import time

# Connect to the drone
# connection_string = '/dev/ttyUSB0'  # Replace with your connection string
connection_string = '127.0.0.1:14550'
vehicle = connect(connection_string)
print('Initialize the drone ...')
vehicle.initialize()

def arm_and_takeoff(target_altitude):
    print("Arming motors...")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off...")
    vehicle.simple_takeoff(target_altitude)

    try:
        while True:
            print(f" Altitude: {vehicle.location.global_relative_frame.alt}")
            if vehicle.location.global_relative_frame.alt >= target_altitude * 0.95:
                print("Reached target altitude")
                break
            time.sleep(1)
    except:
        pass
target_altitude = 10  # Change to your desired altitude
arm_and_takeoff(target_altitude)

# Initialize velocity components
velocity_x = 0
velocity_y = 0
velocity_z = 0

print('Movement commands is listening ...')
while True:
    if keyboard.is_pressed('g'):
        print("MODE == GUIDED")
        vehicle.mode = VehicleMode("GUIDED")

    if keyboard.is_pressed('l'):
        print("MODE == LAND")
        vehicle.mode = VehicleMode("LAND")

    if keyboard.is_pressed('f'):
        print("MODE == AUTO")
        vehicle.mode = VehicleMode("AUTO")

    if keyboard.is_pressed('i'):
        print("MODE == LOITER")
        vehicle.mode = VehicleMode("LOITER")

    if keyboard.is_pressed('w'):
        velocity_x += 1 if not keyboard.is_pressed('shift') else 5
        if velocity_x > 20:
            velocity_x = 20
        print(f"Velocity X Component: {velocity_x}")

    if keyboard.on_release('w'):
        velocity_x = 0
    
    if keyboard.is_pressed('s'):
        velocity_x += -1 if not keyboard.is_pressed('shift') else -5
        if velocity_x < -20:
            velocity_x = -20
        print(f"Velocity X Component: {velocity_x}")

    if keyboard.on_release('s'):
        velocity_x = 0

    if keyboard.is_pressed('a'):
        velocity_y += -1 if not keyboard.is_pressed('shift') else -5
        if velocity_y < -20:
            velocity_y = -20
        print(f"Velocity Y Component: {velocity_y}")

    if keyboard.on_release('a'):
        velocity_y = 0

    if keyboard.is_pressed('d'):
        velocity_y += 1 if not keyboard.is_pressed('shift') else 5
        if velocity_y > 20:
            velocity_y = 20
        print(f"Velocity Y Component: {velocity_y}")

    if keyboard.on_release('d'):
        velocity_y = 0

    if keyboard.is_pressed('e'):
        velocity_z += 1 if not keyboard.is_pressed('shift') else 5
        if velocity_z > 5:
            velocity_z = 5
        print(f"GAINING ALTITUDE {velocity_z}")

    if keyboard.on_release('e'):
        velocity_z = 0

    if keyboard.is_pressed('q'):
        velocity_z += -1 if not keyboard.is_pressed('shift') else -5
        if velocity_z < -5:
            velocity_z = -5
        print(f"LOWERING ALTITUDE {velocity_z}")

    if keyboard.on_release('q'):
        velocity_z = 0

    if keyboard.is_pressed('z'):
        velocity_x = 0
        velocity_y = 0
        velocity_z = 0
        print('ALL VELOCITY COMPONENT RESET')

    if keyboard.is_pressed('x'):
        print('Exiting the program ...')
        try:
            exit(0)
        except:
            # Close the connection
            vehicle.close()
            pass
    # Create a LocationGlobalRelative object with the target coordinates
    target_location = dronekit.LocationGlobalRelative(
        vehicle.location.global_relative_frame.lat + velocity_x,
        vehicle.location.global_relative_frame.lon + velocity_y,
        vehicle.location.global_relative_frame.alt + velocity_z
    )
    
    vehicle.simple_goto(target_location)
    time.sleep(0.05)
