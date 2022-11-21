from pymavlink import mavutil
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import math
import numpy

def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the 
    earth's poles. It comes from the ArduPilot test code: 
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors and
    for the specified duration.

    This uses the SET_POSITION_TARGET_LOCAL_NED command with a type mask enabling only 
    velocity components 
    (http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/#set_position_target_local_ned).
    
    Note that from AC3.3 the message should be re-sent every second (after about 3 seconds
    with no message the velocity will drop back to zero). In AC3.2.1 and earlier the specified
    velocity persists until it is canceled. The code below should work on either version 
    (sending the message multiple times does not cause problems).
    
    See the above link for information on the type_mask (0=enable, 1=ignore). 
    At time of writing, acceleration and yaw bits are ignored.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 

    # send command to vehicle on 1 Hz cycle
    for x in range(0,duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)

def get_theoretical_distance(distance_x, distance_y, distance_z):
    """
    X,Y,Z方向の距離から飛行距離の理論値を計算する。
      distance_x: X方向の距離（単位:m）
      distance_y: Y方向の距離（単位:m）
      distance_z: Z方向の距離（単位:m）
    """
    val = numpy.array([distance_x, distance_y, distance_z])
    print("Distance to theoretical: ", numpy.linalg.norm(val))

def modify_speed(distance_x, distance_y, distance_z, max_speed):
    """
    X,Y,Z方向の最長距離とmax_speedから、X,Y,Z方向の速度と飛行時間を再計算する。
      distance_x: X方向の距離（単位:m）
      distance_y: Y方向の距離（単位:m）
      distance_z: Z方向の距離（単位:m）
      max_speed:  飛行速度（単位:m/s）
    """

    max_distance = max([abs(distance_x), abs(distance_y), abs(distance_z)])
    mod_duration = max_distance / max_speed

    mod_velocity_x = distance_x / mod_duration
    mod_velocity_y = distance_y / mod_duration
    mod_velocity_z = distance_z / mod_duration

    print("modify_speed", mod_velocity_x, mod_velocity_y, mod_velocity_z, mod_duration)
    return mod_velocity_x, mod_velocity_y, mod_velocity_z, mod_duration

def send_flare(velocity_x, velocity_y, velocity_z, duration=0.1):
    """
    フレア操作を実施する。
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x * -1, velocity_y * -1, velocity_z * -1, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    i_duration = int(round(duration,1) * 10)
    # send command to vehicle on 1 Hz cycle
    for x in range(0,i_duration):
        vehicle.send_mavlink(msg)
        time.sleep(0.1)

def send_ned_distance(distance_x, distance_y, distance_z, duration=1, max_speed=1):
    """
    X,Y,Z方向の距離を指定して、飛行させる。
      distance_x: X方向の距離（単位:m）
      distance_y: Y方向の距離（単位:m）
      distance_z: Z方向の距離（単位:m）
      duration:   飛行時間（単位:s）※省略時:1
      max_speed:  飛行速度（単位:m/s）※省略時:1
    """
    get_theoretical_distance(distance_x, distance_y, distance_z)

    if duration > 0:
        velocity_x = distance_x / duration
        velocity_y = distance_y / duration
        velocity_z = distance_z / duration
    
    if duration <= 0 or max([abs(velocity_x), abs(velocity_y), abs(velocity_z)]) > max_speed:
        velocity_x, velocity_y, velocity_z, duration = modify_speed(distance_x, distance_y, distance_z, max_speed)
    else:
        print("original_speed", velocity_x, velocity_y, velocity_z, duration)

    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    i_duration = int(round(duration,1) * 10)
    # send command to vehicle on 1 Hz cycle
    for x in range(0,i_duration):
        vehicle.send_mavlink(msg)
        time.sleep(0.1)

    # フレア操作
    send_flare(velocity_x, velocity_y, velocity_z)

def waitHovering():
    """
    ホバリング状態になるのを待つ。
    """
    while max(vehicle.velocity) >= 0.03:
        time.sleep(0.1)
        print("Groundspeed: %s" % vehicle.groundspeed)
        print("Velocityspeed: %s" % vehicle.velocity)

def get_goto_distance(startLocation):

    waitHovering()
    
    currentLocation=vehicle.location.global_relative_frame
    targetDistance=get_distance_metres(startLocation, currentLocation)
    print("Distance to target: ", targetDistance)

def print_state_velocity(msgStr):
    # vehicleオブジェクト内のステータスを表示
    print("---- %s ----------------------" %  msgStr)
    print(" GPS: %s" % vehicle.gps_0 )
    print(" Battery: %s" % vehicle.battery )
    print(" Last Heartbeat: %s" % vehicle.last_heartbeat )
    print(" Is Armable?: %s" % vehicle.is_armable )
    print(" System status: %s" % vehicle.system_status.state )
    print(" Mode: %s" % vehicle.mode.name )

# MAIN
connection_string = 'tcp:127.0.0.1:5762'
takeoff_alt = 2
vehicle = connect(connection_string, wait_ready=True, timeout=60)

while not vehicle.is_armable:
    time.sleep(1)
vehicle.mode = VehicleMode("GUIDED")
vehicle.armed = True

while not vehicle.armed: 
    print('Waiting for arming...')
    time.sleep(1)
vehicle.simple_takeoff(takeoff_alt) # Take off to target altitude

while True:
    print('Altitude: %d' % vehicle.location.global_relative_frame.alt)
    if vehicle.location.global_relative_frame.alt >= takeoff_alt * 0.95:
     print('REACHED TARGET ALTITUDE') 
     break 
    time.sleep(1) 

# This is the command to move the copter 5 m/s forward for 10 sec.
print_state_velocity("STAET")
startLocation=vehicle.location.global_relative_frame
velocity_x = 5
velocity_y = 0
velocity_z = 0
duration = 2
send_ned_distance(velocity_x, velocity_y, velocity_z, duration)
get_goto_distance(startLocation)
print_state_velocity("END")

# backwards at 5 m/s for 10 sec.
print_state_velocity("STAET")
startLocation=vehicle.location.global_relative_frame
velocity_x = -5
velocity_y = 0
velocity_z = 0
duration = 2
send_ned_distance(velocity_x, velocity_y, velocity_z, duration)
get_goto_distance(startLocation)
print_state_velocity("END")

vehicle.mode = VehicleMode("LAND")
