"""
Applicationコース Day5課題のプログラム
"""

import time
import math
from concurrent.futures import ThreadPoolExecutor
from pymavlink import mavutil
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command

def change_vehicle_mode(vehicle, mode_name, wait_time=5):
    """
    確認機能付きモード変更

    vehicle – 機体オブジェクト
    mode_name – モード名
    wait_time – 変更待ち時間
    """
    while vehicle.mode.name != mode_name:
        vehicle.mode = VehicleMode(mode_name)
        time.sleep(wait_time)
        print("current mode:", vehicle.mode.name )

def get_location_metres(original_location, d_north, d_east):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres
    from the specified `original_location`. The returned LocationGlobal has the same `alt` value
    as `original_location`.

    The function is useful when you want to move the vehicle around specifying locations relative to
    the current vehicle position.

    The algorithm is relatively accurate over small distances (10m within 1km)
    except close to the poles.

    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius = 6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    d_lat = d_north/earth_radius
    d_lon = d_east/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (d_lat * 180/math.pi)
    newlon = original_location.lon + (d_lon * 180/math.pi)
    if isinstance(original_location, LocationGlobal):
        targetlocation=LocationGlobal(newlat, newlon,original_location.alt)
    elif isinstance(original_location, LocationGlobalRelative):
        targetlocation=LocationGlobalRelative(newlat, newlon,original_location.alt)
    else:
        raise Exception("Invalid Location object passed")

    return targetlocation


def get_distance_metres(location1, location2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the
    earth's poles. It comes from the ArduPilot test code:
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = location2.lat - location1.lat
    dlong = location2.lon - location1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

def simple_goto_target(vehicle, lat, lon, alt=None, speed=10):
    """
    simple_goto の到着を確認する

    vehicle – 機体オブジェクト
    lat – 緯度
    lon – 経度
    alt – 平均海面 (MSL) に対するメートル単位の高度(デフォルト=None)
    speed – 速度
    """
    alt_min = 1 #ローバー段差対策

    # モードはGUIDED
    change_vehicle_mode(vehicle, "GUIDED")

    current_location = vehicle.location.global_relative_frame
    target_location = get_location_metres(current_location, lat, lon)
    target_distance = get_distance_metres(current_location, target_location)

    if not alt:
        alt = vehicle.location.global_relative_frame.alt

        if "Rover" in str(vehicle.version) and alt < alt_min:
            alt = alt_min

    # 目標の緯度・経度、高度を設定する
    # https://maps.gsi.go.jp/#8/-35.3574950/149.1701826/&base=std&ls=std&disp=1&vs=c1j0h0k0l0u0t0z0r0s0m0f1
    location = LocationGlobalRelative(lat, lon, alt)

    # simple_gotoを実行する
    print("GoTo Location:",location,", Speed:", speed)
    vehicle.simple_goto(location, groundspeed=speed, airspeed=speed)

    while vehicle.mode.name=="GUIDED": #Stop action if we are no longer in guided mode.
        #print "DEBUG: mode: ", vehicle.mode.name
        remaining_distance=get_distance_metres(vehicle.location.global_relative_frame, location)
        #print("Distance to target: ", remaining_distance)
        if remaining_distance<=target_distance*0.01: #Just below target, in case of undershoot.
            print("Reached target")
            break
        time.sleep(2)

def arming_copter(vehicle):
    """
    Arming

    vehicle – 機体オブジェクト
    """
    while not vehicle.is_armable:
        time.sleep(1)
    change_vehicle_mode(vehicle, "GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print('Waiting for arming...')
        time.sleep(1)

def takeoff_copter(vehicle):
    """
    Take off

    vehicle – 機体オブジェクト
    """
    takeoff_alt = 10

    arming_copter(vehicle)

    vehicle.simple_takeoff(takeoff_alt) # Take off to target altitude

    if "Copter" in str(vehicle.version):
        while True:
            print('Altitude: ', str(vehicle.location.global_relative_frame.alt))
            if vehicle.location.global_relative_frame.alt >= takeoff_alt * 0.95:
                print('REACHED TARGET ALTITUDE')
                break
            time.sleep(1)

    print("離陸")

def land_copter(vehicle):
    """
    Land and Disarming

    vehicle – 機体オブジェクト
    """
    if "Copter" in str(vehicle.version):
        change_vehicle_mode(vehicle, "LAND")
    else:
        vehicle.disarm()

    while vehicle.armed:
        print('Waiting for disarming...')
        time.sleep(1)

    print("着陸")

def loop_flight(vehicle, list_points, land_time=20, loop_num=0, speed=10):
    """
    指定された座標リストを周回する

    vehicle – 機体オブジェクト
    list_points – 座標リスト
    land_time – 着陸時間
    loop_num - ループ回数（例：往復 = 2、無限ループ = 0）
    speed - 速度
    """
    fly_points = list_points.copy()
    num = 0
    while True:
        num = num + 1

        takeoff_copter(vehicle)

        for point in fly_points[1:]:
            lat, lon = point
            simple_goto_target(vehicle, lat, lon, None, speed)

        land_copter(vehicle)

        if loop_num != 0 and num >= loop_num:
            break

        time.sleep(land_time)
        fly_points.reverse()

def readmission(file_name):
    """
    Load a mission from a file into a list.
    This function is used by upload_mission().

    file_name – ミッションファイル
    """
    print("Reading mission from file:", file_name)

    missionlist=[]
    with open(file_name, "r", encoding='utf_8') as fileobj:
        for i, line in enumerate(fileobj):
            if i==0:
                if not line.startswith('QGC WPL 110'):
                    raise Exception('File is not supported WP version')
            else:
                linearray=line.split('\t')
                #ln_index=int(linearray[0])
                ln_currentwp=int(linearray[1])
                ln_frame=int(linearray[2])
                ln_command=int(linearray[3])
                ln_param1=float(linearray[4])
                ln_param2=float(linearray[5])
                ln_param3=float(linearray[6])
                ln_param4=float(linearray[7])
                ln_param5=float(linearray[8])
                ln_param6=float(linearray[9])
                ln_param7=float(linearray[10])
                ln_autocontinue=int(linearray[11].strip())

                cmd = Command( 0, 0, 0, ln_frame, ln_command, ln_currentwp, ln_autocontinue,
                        ln_param1, ln_param2, ln_param3, ln_param4, ln_param5, ln_param6, ln_param7)
                missionlist.append(cmd)

    return missionlist

def upload_mission(vehicle, import_mission_filename):
    """
    Upload a mission from a file.

    vehicle – 機体オブジェクト
    import_mission_filename – ミッションファイル
    """
    #Read mission from file
    print("\nUpload mission from a file: ", import_mission_filename)
    missionlist = readmission(import_mission_filename)

    #Clear existing mission from vehicle
    print('> Clear mission')
    cmds = vehicle.commands
    cmds.clear()

    print("> Add new mission to vehicle")
    for command in missionlist:
        cmds.add(command)

    print('> Upload mission')
    vehicle.commands.upload()

def upload_takeoff_mission(vehicle):
    """
    Upload a Take off mission.

    vehicle – 機体オブジェクト
    """
    # コマンドオブジェクトの取得
    cmds = vehicle.commands

    # ダウンロード実行
    cmds.download()
    cmds.wait_ready()

    # コマンド定義（テイクオフミッションを定義）
    cmd = Command(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, 0, 10
    )

    # コマンド追加
    cmds.clear()
    cmds.add(cmd)
    # ミッションアップロード
    cmds.upload()

def get_all_vehicle_state(vehicle):
    """
    Get all vehicle attributes (state)
    """
    print("\nGet all vehicle attribute values:")
    print(" Autopilot Firmware version:", vehicle.version)
    print("   Major version number:", vehicle.version.major)
    print("   Minor version number:", vehicle.version.minor)
    print("   Patch version number:", vehicle.version.patch)
    print("   Release type:", vehicle.version.release_type())
    print("   Release version:", vehicle.version.release_version())
    print("   Stable release?:", vehicle.version.is_stable())
    print(" Autopilot capabilities")
    print("   Supports MISSION_FLOAT message type:", vehicle.capabilities.mission_float)
    print("   Supports PARAM_FLOAT message type:", vehicle.capabilities.param_float)
    print("   Supports MISSION_INT message type:", vehicle.capabilities.mission_int)
    print("   Supports COMMAND_INT message type:", vehicle.capabilities.command_int)
    print("   Supports PARAM_UNION message type:", vehicle.capabilities.param_union)
    print("   Supports ftp for file transfers:", vehicle.capabilities.ftp)
    print("   Supports commanding attitude offboard:", vehicle.capabilities.set_attitude_target)
    print("   Supports commanding position and velocity targets in local NED frame:",
     vehicle.capabilities.set_attitude_target_local_ned)
    print("   Supports set position + velocity targets in global scaled integers:",
     vehicle.capabilities.set_altitude_target_global_int)
    print("   Supports terrain protocol / data handling:", vehicle.capabilities.terrain)
    print("   Supports direct actuator control:", vehicle.capabilities.set_actuator_target)
    print("   Supports the flight termination command:", vehicle.capabilities.flight_termination)
    print("   Supports mission_float message type:", vehicle.capabilities.mission_float)
    print("   Supports onboard compass calibration:", vehicle.capabilities.compass_calibration)
    print(" Global Location:", vehicle.location.global_frame)
    print(" Global Location (relative altitude):", vehicle.location.global_relative_frame)
    print(" Local Location:", vehicle.location.local_frame)
    print(" Attitude:", vehicle.attitude)
    print(" Velocity:", vehicle.velocity)
    print(" GPS:", vehicle.gps_0)
    print(" Gimbal status:", vehicle.gimbal)
    print(" Battery:", vehicle.battery)
    print(" EKF OK?:", vehicle.ekf_ok)
    print(" Last Heartbeat:", vehicle.last_heartbeat)
    print(" Rangefinder:", vehicle.rangefinder)
    print(" Rangefinder distance:", vehicle.rangefinder.distance)
    print(" Rangefinder voltage:", vehicle.rangefinder.voltage)
    print(" Heading:", vehicle.heading)
    print(" Is Armable?:", vehicle.is_armable)
    print(" System status:", vehicle.system_status.state)
    print(" Groundspeed:", vehicle.groundspeed)    # settable
    print(" Airspeed:", vehicle.airspeed)    # settable
    print(" Mode:", vehicle.mode.name)    # settable
    print(" Armed:", vehicle.armed)    # settable

def copter(vehicle):
    """
    コプター運航 ルート: メインポート ⇔ 隣接ポート

    vehicle – 機体オブジェクト
    """
    copter_route_points = [[35.878275, 140.338069],[35.867003, 140.305987]]

    #コプター： 隣接ポート ⇔ メインポート 往復
    loop_flight(vehicle, copter_route_points, 20, 0)

def rover1(vehicle_r1, vehicle_c):
    """
    ローバー１運行 ルート: 隣接ポート ⇔ セブンイレブン（隣接ポート）

    vehicle_r1 – ローバー１機体オブジェクト
    vehicle_c – コプター機体オブジェクト
    """
    rover1_route_points = [
                            [35.867003, 140.305987],
                            [35.8692646, 140.2978992],
                            [35.8723553, 140.2910730],
                            [35.8776420, 140.2949530],
                            [35.877518, 140.295439]]

    while True:
        while True:
            # コプターと隣接ポートの距離 (m)
            port_location = LocationGlobalRelative(35.867003, 140.305987, 0)
            copter_port_distance = \
                get_distance_metres(vehicle_c.location.global_relative_frame, port_location)
            print("Copter distance to port: ", copter_port_distance, vehicle_c.mode.name)

            if not vehicle_r1.armed and not vehicle_c.armed and copter_port_distance < 1:
                print("Copter reached port")
                break
            time.sleep(2)

        #ローバー１: 隣接ポート ⇔ セブンイレブン（隣接ポート）往復
        loop_flight(vehicle_r1, rover1_route_points, 20, 2)
        time.sleep(20)

def rover2(vehicle_r2, vehicle_b1, vehicle_b2):
    """
    ローバー２運行 ルート: 対岸ポート ⇔ 滑川駅（対岸ポート）

    vehicle_r2 – ローバー１機体オブジェクト
    vehicle_b1 – ボート１機体オブジェクト
    vehicle_b2 – ボート２機体オブジェクト
    """
    rover2_route_points = [
                            [35.879768, 140.348495],
                            [35.8787907, 140.3496203],
                            [35.876991, 140.348026]]
    loop_num = 0
    while True:
        while True:
            # ボートと対岸ポートの距離 (m)
            port_location = LocationGlobalRelative(35.8802285 ,140.3476155, 0)
            boat1_port_distance = \
                get_distance_metres(vehicle_b1.location.global_relative_frame, port_location)
            boat2_port_distance = \
                get_distance_metres(vehicle_b2.location.global_relative_frame, port_location)
            print("Boat1,2 distance to port: ", boat1_port_distance, boat2_port_distance)

            if not vehicle_r2.armed and \
                ((not vehicle_b1.armed and boat1_port_distance < 1) or \
                 (not vehicle_b2.armed and boat2_port_distance < 1)):
                print("Boat1 or 2 reached port")
                loop_num += 1
                break
            time.sleep(2)

        if loop_num > 1:
            #ローバー１: 隣接ポート ⇔ セブンイレブン（隣接ポート）往復
            loop_flight(vehicle_r2, rover2_route_points, 20, 2, 15)
        time.sleep(20)

def boat1(vehicle):
    """
    ボート１運航 ルート: メインポート ⇔ 対岸ポート

    vehicle – 機体オブジェクト
    """
    boat1_route_points = [[35.8773498,140.3383255],
                          [35.8801837, 140.3436470],
                          [35.8802285 ,140.3476155]]

    #ボート１： メインポート ⇔ 対岸ポート 往復
    loop_flight(vehicle, boat1_route_points, 60, 0, 8)

def boat2(vehicle):
    """
    ボート２運航 ルート: 対岸ポート ⇔ メインポート

    vehicle – 機体オブジェクト
    """
    boat2_route_points = [[35.8802285 ,140.3476155],
                          [35.8783582, 140.3436685],
                          [35.8773498,140.3383255]]

    #ボート２： 対岸ポート ⇔ メインポート 往復
    loop_flight(vehicle, boat2_route_points, 60, 0, 8)

def plane(vehicle):
    """
    プレーン運航 ルート: メインポートで離陸、監視のため指定河川上空を往復

    vehicle – 機体オブジェクト
    """
    #change_vehicle_mode(vehicle, "GUIDED")
    arming_copter(vehicle)
    #takeoff_copter(vehicle)

    #upload_takeoff_mission(vehicle)
    #upload_mission(vehicle, "./takeoff.waypoints")
    #change_vehicle_mode(vehicle, "AUTO")

    change_vehicle_mode(vehicle, "GUIDED")
    upload_mission(vehicle, "./plane.waypoints")
    change_vehicle_mode(vehicle, "TAKEOFF")

    takeoff_alt = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
    while True:
        print('Altitude:', str(vehicle.location.global_relative_frame.alt))
        if vehicle.location.global_relative_frame.alt >= takeoff_alt * 0.95:
            print('REACHED TARGET ALTITUDE')
            break
        time.sleep(1)

    while True:
        change_vehicle_mode(vehicle, "AUTO")

        while vehicle.commands.next < 12:
            time.sleep(2)

        change_vehicle_mode(vehicle, "GUIDED")
        vehicle.commands.next = 1

# MAIN
vehicle_copter = connect('tcp:127.0.0.1:5773', wait_ready=True, timeout=60)
vehicle_rover1 = connect('tcp:127.0.0.1:5783', wait_ready=True, timeout=60)
vehicle_rover2 = connect('tcp:127.0.0.1:5793', wait_ready=True, timeout=60)
vehicle_boat1 = connect('tcp:127.0.0.1:5803', wait_ready=True, timeout=60)
vehicle_boat2 = connect('tcp:127.0.0.1:5813', wait_ready=True, timeout=60)
vehicle_plane = connect('tcp:127.0.0.1:5823', wait_ready=True, timeout=60)

#メインポート：35.878275, 140.338069
#対岸ポート：35.879768, 140.348495
#隣接ポート：35.867003, 140.305987
#対岸ポートの配送先 滑川駅（対岸ポート）：35.876991, 140.348026
#隣接ポートの配送先 セブンイレブン（隣接ポート）：35.877518, 140.295439

with ThreadPoolExecutor() as executor:
    feature_copter = executor.submit(copter,vehicle_copter)
    feature_rover1 = executor.submit(rover1,vehicle_rover1,vehicle_copter)
    feature_boat1 = executor.submit(boat1,vehicle_boat1)
    feature_boat2 = executor.submit(boat2,vehicle_boat2)
    feature_rover2 = executor.submit(rover2,vehicle_rover2,vehicle_boat1,vehicle_boat2)
    feature_plane = executor.submit(plane,vehicle_plane)
