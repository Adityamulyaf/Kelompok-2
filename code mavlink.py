import collections
try:
    from collections import abc
    print("READY")
    collections.MutableMapping = abc.MutableMapping
except:
    pass

from dronekit import connect, VehicleMode, Command
from pymavlink import mavutil
import time

print('Connecting...')
# vehicle = connect('tcp:192.168.229.171:5762')
vehicle = connect('tcp:127.0.0.1:5762')
# vehicle = connect('/dev/ttyACM0')
# vehicle = connect('tcp:127.0.0.1:5762', wait_ready=True)

print("WAITING..")

def arm_and_takeoff(altitude):
    while not vehicle.is_armable:
        print("waiting to be armable")
        # time.sleep(1)
    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    print("Taking Off")
    vehicle.simple_takeoff(altitude)

    while True:
        v_alt = vehicle.location.global_relative_frame.alt
        print(">> Altitude = %.1f m" % v_alt)
        if v_alt >= altitude - 1.0:
            print("Target altitude reached")
            break


def add_last_waypoint_to_mission(  wp_Last_Latitude,
        wp_Last_Longitude,
        wp_Last_Altitude):  # --- [m]    Target Altitude
    """
    Upload the mission with the last WP as given and outputs the ID to be set
    """
    # Get the set of commands from the vehicle
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()

    # Save the vehicle commands to a list
    missionlist = []
    for cmd in cmds:
        missionlist.append(cmd)

    # Modify the mission as needed. For example, here we change the
    wpLastObject = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                           0, 0, 0, 0, 0, 0,
                           wp_Last_Latitude, wp_Last_Longitude, wp_Last_Altitude)
    missionlist.append(wpLastObject)

    # Clear the current mission (command is sent when we call upload())
    cmds.clear()

    # Write the modified mission and flush to the vehicle
    for cmd in missionlist:
        cmds.add(cmd)
    cmds.upload()

    return (cmds.count)


def gerak(vx, vy):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        time,
        0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0b0000111111000111,
        0, 0, 0,
        vx, vy, 0,
        0, 0, 0,
        0, 0)
    vehicle.send_mavlink(msg)
    vehicle.flush()


if __name__ == "__main__":
    add_last_waypoint_to_mission(vehicle.location.global_relative_frame.lat,
                                 vehicle.location.global_relative_frame.lon,
                                 vehicle.location.global_relative_frame.alt)
    print("Home waypoint added to the mission")
    arm_and_takeoff(0)
    print("READY")
    ground_speed = 2
    while True :
        gerak(ground_speed, 0)
        print("GOO")
    