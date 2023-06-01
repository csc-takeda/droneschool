#! /usr/bin/python3 

import rospy
from sensor_msgs.msg import LaserScan

def laser_scan_callback(msg):
    thin_out_num = 7
    obstacle_distance_msg = LaserScan()
    obstacle_distance_msg.header = msg.header
    obstacle_distance_msg.angle_min = msg.angle_min
    obstacle_distance_msg.angle_max = msg.angle_max
    obstacle_distance_msg.angle_increment = msg.angle_increment
    obstacle_distance_msg.time_increment = msg.time_increment
    obstacle_distance_msg.scan_time = msg.scan_time
    obstacle_distance_msg.range_min = msg.range_min
    obstacle_distance_msg.range_max = msg.range_max
    obstacle_distance_msg.ranges = msg.ranges[::-1]

    laser_scan_pub.publish(obstacle_distance_msg)

rospy.init_node('laser_scan_to_obstacle_distance')
laser_scan_pub = rospy.Publisher('/mavros/obstacle/send', LaserScan, queue_size=1)
rospy.Subscriber('/scan', LaserScan, laser_scan_callback)
rospy.spin()
