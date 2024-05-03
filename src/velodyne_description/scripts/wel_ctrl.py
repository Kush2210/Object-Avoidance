from dronekit import connect, VehicleMode, LocationGlobalRelative, mavutil

import time
import numpy as np
import math
from math import radians, cos, sin, asin, sqrt

vehicle = connect('127.0.0.1:14550', wait_ready=False)


def send_ned_velocity(velocity_x, velocity_y, velocity_z):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,  # time_boot_ms (not used)
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
        0b0000111111000111,  # type_mask (only speeds enabled)
        0, 0, 0,  # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z,  # x, y, z velocity in m/s
        0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    vehicle.send_mavlink(msg)

while True:
    vel_x = 3
    vel_y = 3
    send_ned_velocity(vel_x, vel_y, 0)


