import threading

import mavros_msgs.msg
import mavros_msgs.srv
import rospy

import time
from mavros_python_examples.roverHandler import *
from mavros_msgs.msg import GlobalPositionTarget
from mavros_msgs.msg import PositionTarget
from mavros_msgs.msg import VFR_HUD
from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3
from mavros_python_examples.topicService import TopicService

class MyRoverHandler(RoverHandler):
    def __init__(self):
        super().__init__()

        self.TOPIC_SET_POSE_LOCAL = TopicService('/mavros/setpoint_raw/local', PositionTarget)
        self.TOPIC_SET_VFR_HUD = TopicService('/mavros/vfr_hud', VFR_HUD)

        self.local_pos = Point(0.0, 0.0, 0.0)
        self.local_speed = 0.0

        self.user_thread = threading.Timer(0, self.user)
        self.user_thread.daemon = True
        self.user_thread.start()

    def move_pos(self, x: float, y: float, z: float = 0.0):
        data = PositionTarget()
        pos = Point()
        pos.x = x
        pos.y = y
        pos.z = z
        data.position = pos
        data.coordinate_frame = 9
        data.type_mask = 3576
        print("Set position:", pos)
        self.local_pos.x += pos.x
        self.local_pos.y += pos.y
        print("Move position:", self.local_pos)
        rospy.Subscriber('/mavros/vfr_hud', VFR_HUD, self.cb_groundspeed)
        while self.local_speed <= 0.03:
            self.TOPIC_SET_POSE_LOCAL.set_data(data)
            self.topic_publisher(topic=self.TOPIC_SET_POSE_LOCAL)
            rospy.sleep(1)
            rospy.Subscriber('/mavros/vfr_hud', VFR_HUD, self.cb_groundspeed)
        else:
            self.TOPIC_SET_POSE_LOCAL.set_data(data)
            self.topic_publisher(topic=self.TOPIC_SET_POSE_LOCAL)

    def cb_groundspeed(self, data):
         self.local_speed = data.groundspeed

    def to_arm(self):
        while True:
            rospy.sleep(1)
            print("arm:", self.armed, "mode:", self.mode)
            print("set param:", self.set_param("CRUISE_SPEED", 2, 0))
            if self.connected:
                print("get param:", self.get_param("CRUISE_SPEED"))
                self.change_mode(MODE_GUIDED)
                self.arm(True)
                break

    def wait_rover_stop(self):
        rospy.sleep(1)
        rospy.Subscriber('/mavros/vfr_hud', VFR_HUD, self.cb_groundspeed)
        print("Groundspeed0: %s" % self.local_speed)
        while self.local_speed >= 0.03:
            rospy.sleep(0.1)
            rospy.Subscriber('/mavros/vfr_hud', VFR_HUD, self.cb_groundspeed)
            print("Groundspeed: %s" % self.local_speed)

    def user(self):
        # Setpoint publisher
        rospy.sleep(0.2)
        self.to_arm()
        rospy.sleep(1)

        self.move_pos(5.0, 0.0)
        self.wait_rover_stop()

        self.move_pos(0.0, 5.0)
        self.wait_rover_stop()

        self.move_pos(-5.0, 5.0)
        self.wait_rover_stop()

        rospy.signal_shutdown(self)

if __name__ == "__main__":
    v = MyRoverHandler()
    v.enable_topics_for_read()
    v.connect("node1", rate=10)

