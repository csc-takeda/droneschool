from pymavlink import mavutil
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
from pymavlink import mavutil
import time
import math
import numpy
from concurrent.futures import ThreadPoolExecutor

def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
    specified `original_location`. The returned LocationGlobal has the same `alt` value
    as `original_location`.

    The function is useful when you want to move the vehicle around specifying locations relative to 
    the current vehicle position.

    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.

    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius = 6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    if type(original_location) is LocationGlobal:
        targetlocation=LocationGlobal(newlat, newlon,original_location.alt)
    elif type(original_location) is LocationGlobalRelative:
        targetlocation=LocationGlobalRelative(newlat, newlon,original_location.alt)
    else:
        raise Exception("Invalid Location object passed")
        
    return targetlocation;


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

def simple_goto_target(vehicle, lat, lon, alt=None, speed=10):
    """
    simple_goto の到着を確認する
    
    vehicle – 機体オブジェクト
    lat – 緯度
    lon – 経度
    alt – 平均海面 (MSL) に対するメートル単位の高度(デフォルト=None)
    speed – 速度
    """
    
    alt_min = 1    #ローバー段差対策
    
    # モードはGUIDED
    vehicle.mode = VehicleMode("GUIDED")
    
    currentLocation = vehicle.location.global_relative_frame
    targetLocation = get_location_metres(currentLocation, lat, lon)
    targetDistance = get_distance_metres(currentLocation, targetLocation)
    
    if not alt:
        alt = vehicle.location.global_relative_frame.alt
        
        if "Rover" in str(vehicle.version) and alt < alt_min:
            alt = alt_min
    
    # 目標の緯度・経度、高度を設定する
    # https://maps.gsi.go.jp/#8/-35.3574950/149.1701826/&base=std&ls=std&disp=1&vs=c1j0h0k0l0u0t0z0r0s0m0f1
    aLocation = LocationGlobalRelative(lat, lon, alt)
    
    # simple_gotoを実行する
    print("GoTo Location:",aLocation,", Speed:", speed)
    vehicle.simple_goto(aLocation, groundspeed=speed, airspeed=speed)
    
    while vehicle.mode.name=="GUIDED": #Stop action if we are no longer in guided mode.
        #print "DEBUG: mode: %s" % vehicle.mode.name
        remainingDistance=get_distance_metres(vehicle.location.global_relative_frame, aLocation)
        #print("Distance to target: ", remainingDistance)
        if remainingDistance<=targetDistance*0.01: #Just below target, in case of undershoot.
            print("Reached target")
            break;
        time.sleep(2)

def arming_copter(vehicle):
    """
    Arming
    
    vehicle – 機体オブジェクト
    """
    while not vehicle.is_armable:
        time.sleep(1)
    vehicle.mode = VehicleMode("GUIDED")
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
            print('Altitude: %d' % vehicle.location.global_relative_frame.alt)
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
        while vehicle.mode.name != "LAND":
            vehicle.mode = VehicleMode("LAND")
            time.sleep(5)
            print(" Mode: %s" % vehicle.mode.name )
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
    n = 0
    while True:
        n = n + 1
        
        takeoff_copter(vehicle)
        
        for point in fly_points[1:]:
            lat, lon = point
            simple_goto_target(vehicle, lat, lon, None, speed)
        
        land_copter(vehicle)
        
        if loop_num != 0 and n >= loop_num:
            break
        
        time.sleep(land_time)
        fly_points.reverse()

def readmission(vehicle, aFileName):
    """
    Load a mission from a file into a list.
    This function is used by upload_mission().
    
    vehicle – 機体オブジェクト
    aFileName – ミッションファイル
    """
    print("Reading mission from file: %s\n" % aFileName)
    cmds = vehicle.commands
    missionlist=[]
    with open(aFileName) as f:
        for i, line in enumerate(f):
            if i==0:
                if not line.startswith('QGC WPL 110'):
                    raise Exception('File is not supported WP version')
            else:
                linearray=line.split('\t')
                ln_index=int(linearray[0])
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
                
                cmd = Command( 0, 0, 0, ln_frame, ln_command, ln_currentwp, ln_autocontinue, ln_param1, ln_param2, ln_param3, ln_param4, ln_param5, ln_param6, ln_param7)
                missionlist.append(cmd)
                
    return missionlist

def upload_mission(vehicle, import_mission_filename):
    """
    Upload a mission from a file.
    
    vehicle – 機体オブジェクト
    import_mission_filename – ミッションファイル
    """
    #Read mission from file
    print("\nUpload mission from a file: %s" % import_mission_filename)
    missionlist = readmission(vehicle, import_mission_filename)
    
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
    print(" Autopilot Firmware version: %s" % vehicle.version)
    print("   Major version number: %s" % vehicle.version.major)
    print("   Minor version number: %s" % vehicle.version.minor)
    print("   Patch version number: %s" % vehicle.version.patch)
    print("   Release type: %s" % vehicle.version.release_type())
    print("   Release version: %s" % vehicle.version.release_version())
    print("   Stable release?: %s" % vehicle.version.is_stable())
    print(" Autopilot capabilities")
    print("   Supports MISSION_FLOAT message type: %s" % vehicle.capabilities.mission_float)
    print("   Supports PARAM_FLOAT message type: %s" % vehicle.capabilities.param_float)
    print("   Supports MISSION_INT message type: %s" % vehicle.capabilities.mission_int)
    print("   Supports COMMAND_INT message type: %s" % vehicle.capabilities.command_int)
    print("   Supports PARAM_UNION message type: %s" % vehicle.capabilities.param_union)
    print("   Supports ftp for file transfers: %s" % vehicle.capabilities.ftp)
    print("   Supports commanding attitude offboard: %s" % vehicle.capabilities.set_attitude_target)
    print("   Supports commanding position and velocity targets in local NED frame: %s" % vehicle.capabilities.set_attitude_target_local_ned)
    print("   Supports set position + velocity targets in global scaled integers: %s" % vehicle.capabilities.set_altitude_target_global_int)
    print("   Supports terrain protocol / data handling: %s" % vehicle.capabilities.terrain)
    print("   Supports direct actuator control: %s" % vehicle.capabilities.set_actuator_target)
    print("   Supports the flight termination command: %s" % vehicle.capabilities.flight_termination)
    print("   Supports mission_float message type: %s" % vehicle.capabilities.mission_float)
    print("   Supports onboard compass calibration: %s" % vehicle.capabilities.compass_calibration)
    print(" Global Location: %s" % vehicle.location.global_frame)
    print(" Global Location (relative altitude): %s" % vehicle.location.global_relative_frame)
    print(" Local Location: %s" % vehicle.location.local_frame)
    print(" Attitude: %s" % vehicle.attitude)
    print(" Velocity: %s" % vehicle.velocity)
    print(" GPS: %s" % vehicle.gps_0)
    print(" Gimbal status: %s" % vehicle.gimbal)
    print(" Battery: %s" % vehicle.battery)
    print(" EKF OK?: %s" % vehicle.ekf_ok)
    print(" Last Heartbeat: %s" % vehicle.last_heartbeat)
    print(" Rangefinder: %s" % vehicle.rangefinder)
    print(" Rangefinder distance: %s" % vehicle.rangefinder.distance)
    print(" Rangefinder voltage: %s" % vehicle.rangefinder.voltage)
    print(" Heading: %s" % vehicle.heading)
    print(" Is Armable?: %s" % vehicle.is_armable)
    print(" System status: %s" % vehicle.system_status.state)
    print(" Groundspeed: %s" % vehicle.groundspeed)    # settable
    print(" Airspeed: %s" % vehicle.airspeed)    # settable
    print(" Mode: %s" % vehicle.mode.name)    # settable
    print(" Armed: %s" % vehicle.armed)    # settable

def copter(vehicle):
    """
    コプター運航 ルート: メインポート ⇔ 隣接ポート
    
    vehicle – 機体オブジェクト
    """
    copter_route_points = [[35.878275, 140.338069],[35.867003, 140.305987]]
    
    #コプター： 隣接ポート ⇔ メインポート 往復
    loop_flight(vehicle, copter_route_points, 20, 0)

def rover1(vehicleR1, vehicleC):
    """
    ローバー１運行 ルート: 隣接ポート ⇔ セブンイレブン（隣接ポート）
    
    vehicleR1 – ローバー１機体オブジェクト
    vehicleC – コプター機体オブジェクト
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
            portLocation = LocationGlobalRelative(35.867003, 140.305987, 0)
            copter_portDistance = get_distance_metres(vehicleC.location.global_relative_frame, portLocation)
            print("Copter distance to port: ", copter_portDistance, vehicleC.mode.name)
            
            if not vehicleR1.armed and not vehicleC.armed and copter_portDistance < 1:
                print("Copter reached port")
                break;
            time.sleep(2)
        
        #ローバー１: 隣接ポート ⇔ セブンイレブン（隣接ポート）往復
        loop_flight(vehicleR1, rover1_route_points, 20, 2)
        time.sleep(20)

def rover2(vehicleR2, vehicleB1, vehicleB2):
    """
    ローバー２運行 ルート: 対岸ポート ⇔ 滑川駅（対岸ポート）
    
    vehicleR2 – ローバー１機体オブジェクト
    vehicleB1 – ボート１機体オブジェクト
    vehicleB2 – ボート２機体オブジェクト
    """
    rover2_route_points = [
                            [35.879768, 140.348495],
                            [35.8787907, 140.3496203],
                            [35.876991, 140.348026]]
    loop_num = 0
    while True:
        while True:
            # ボートと対岸ポートの距離 (m)
            portLocation = LocationGlobalRelative(35.8802285 ,140.3476155, 0)
            boat1_portDistance = get_distance_metres(vehicleB1.location.global_relative_frame, portLocation)
            boat2_portDistance = get_distance_metres(vehicleB2.location.global_relative_frame, portLocation)
            print("Boat1,2 distance to port: ", boat1_portDistance, boat2_portDistance)
            
            if not vehicleR2.armed and (not vehicleB1.armed and boat1_portDistance < 1) or (not vehicleB2.armed and boat2_portDistance < 1):
                print("Boat1 or 2 reached port")
                loop_num += 1
                break;
            time.sleep(2)
        
        if loop_num > 1:
            #ローバー１: 隣接ポート ⇔ セブンイレブン（隣接ポート）往復
            loop_flight(vehicleR2, rover2_route_points, 20, 2, 15)
        time.sleep(20)

def boat1(vehicle):
    """
    ボート１運航 ルート: メインポート ⇔ 対岸ポート
    
    vehicle – 機体オブジェクト
    """
    boat1_route_points = [[35.8773498,140.3383255],[35.8801837, 140.3436470],[35.8802285 ,140.3476155]]
    
    #ボート１： メインポート ⇔ 対岸ポート 往復
    loop_flight(vehicle, boat1_route_points, 60, 0, 8)

def boat2(vehicle):
    """
    ボート２運航 ルート: 対岸ポート ⇔ メインポート
    
    vehicle – 機体オブジェクト
    """
    boat2_route_points = [[35.8802285 ,140.3476155],[35.8783582, 140.3436685],[35.8773498,140.3383255]]
    
    #ボート２： 対岸ポート ⇔ メインポート 往復
    loop_flight(vehicle, boat2_route_points, 60, 0, 8)

def plane(vehicle):
    """
    プレーン運航 ルート: メインポートで離陸、監視のため指定河川上空を往復
    
    vehicle – 機体オブジェクト
    """
    #vehicle.mode = VehicleMode("GUIDED")
    arming_copter(vehicle)
    #takeoff_copter(vehicle)
    
    #upload_takeoff_mission(vehicle)
    #upload_mission(vehicle, "./takeoff.waypoints")
    #vehicle.mode = VehicleMode("AUTO")
    
    vehicle.mode = VehicleMode("GUIDED")
    upload_mission(vehicle, "./plane.waypoints")
    vehicle.mode = VehicleMode("TAKEOFF")
    
    takeoff_alt = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
    while True:
        print('Altitude: %d' % vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= takeoff_alt * 0.95:
            print('REACHED TARGET ALTITUDE') 
            break 
        time.sleep(1) 
    
    while True:
        vehicle.mode = VehicleMode("AUTO")
        
        while vehicle.commands.next < 12:
            time.sleep(2)
        
        vehicle.mode = VehicleMode("GUIDED")
        vehicle.commands.next = 1

# MAIN
vehicleC = connect('tcp:127.0.0.1:5773', wait_ready=True, timeout=60)
vehicleR1 = connect('tcp:127.0.0.1:5783', wait_ready=True, timeout=60)
vehicleR2 = connect('tcp:127.0.0.1:5793', wait_ready=True, timeout=60)
vehicleB1 = connect('tcp:127.0.0.1:5803', wait_ready=True, timeout=60)
vehicleB2 = connect('tcp:127.0.0.1:5813', wait_ready=True, timeout=60)
vehicleP = connect('tcp:127.0.0.1:5823', wait_ready=True, timeout=60)

#メインポート：35.878275, 140.338069
#対岸ポート：35.879768, 140.348495
#隣接ポート：35.867003, 140.305987
#対岸ポートの配送先 滑川駅（対岸ポート）：35.876991, 140.348026
#隣接ポートの配送先 セブンイレブン（隣接ポート）：35.877518, 140.295439

with ThreadPoolExecutor() as executor:
    featureC = executor.submit(copter,vehicleC)
    featureR1 = executor.submit(rover1,vehicleR1,vehicleC)
    featureB1 = executor.submit(boat1,vehicleB1)
    featureB2 = executor.submit(boat2,vehicleB2)
    featureR2 = executor.submit(rover2,vehicleR2,vehicleB1,vehicleB2)
    featureP = executor.submit(plane,vehicleP)

