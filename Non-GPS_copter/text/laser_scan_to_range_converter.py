#! /usr/bin/python3 

import rospy
from sensor_msgs.msg import LaserScan, Range

def process_laser_scan(msg):
    angle_min = msg.angle_min
    angle_increment = msg.angle_increment
    
    # レンジデータの取得
    ranges = msg.ranges

    # スキャンポイントごとに処理
    for i, range_value in enumerate(ranges):
        # 現在の角度を計算
        angle = angle_min + i * angle_increment

        # Rangeメッセージの作成
        range_msg = Range()
        range_msg.header = msg.header
        range_msg.radiation_type = Range.INFRARED
        range_msg.field_of_view = angle
        range_msg.min_range = msg.range_min
        range_msg.max_range = msg.range_max
        range_msg.range = range_value

        # 例: 角度とレンジデータの表示
        #print("Angle:", angle)
        #print("Range:", range_value)

        # Rangeメッセージのパブリッシュ
        if range_msg.range != 0.0 and (range_msg.min_range <= range_msg.range and range_msg.range <= range_msg.max_range):
            range_pub.publish(range_msg)

# LaserScanメッセージを受信したときに呼び出されるコールバック関数
def laser_scan_callback(msg):
    process_laser_scan(msg)

rospy.init_node('laser_scan_to_range_converter')
range_pub = rospy.Publisher('converted_range', Range, queue_size=10)
laser_scan_sub = rospy.Subscriber('scan', LaserScan, laser_scan_callback)

rospy.spin()
