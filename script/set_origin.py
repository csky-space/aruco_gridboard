#!/usr/bin/env python
import math

import mavros_msgs.msg
##
#
# Send SET_GPS_GLOBAL_ORIGIN and SET_HOME_POSITION messages
#
##
import rospy
from pymavlink.dialects.v10 import ardupilotmega as MAV_APM
from mavros.mavlink import convert_to_rosmsg
from mavros_msgs.msg import Mavlink, LandingTarget, StatusText
from geometry_msgs.msg import PoseStamped, Vector3
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import Range
from tf import transformations
from std_msgs.msg import Int8
import numpy


# Global position of the origin
lat = 53.56335 * 1e7   # Terni
lon = 39.64329 * 1e7   # Terni
alt = 163 * 1e3

mav_pub: rospy.topics.Publisher
global landing_pub
status_text_pub: rospy.topics.Publisher
my_publisher: rospy.topics.Publisher
rangefinder: rospy.topics.Publisher

vp_status: Int8 = None

class fifo(object):
    """ A simple buffer """
    def __init__(self):
        self.buf = []
    def write(self, data):
        self.buf += data
        return len(data)
    def read(self):
        return self.buf.pop(0)

def send_message(msg, mav, pub):
    """
    Send a mavlink message
    """
    msg.pack(mav)
    rosmsg = convert_to_rosmsg(msg)
    pub.publish(rosmsg)

    print("sent message %s" % msg)

def set_global_origin(mav, pub):
    """
    Send a mavlink SET_GPS_GLOBAL_ORIGIN message, which allows us
    to use local position information without a GPS.
    """
    target_system = mav.srcSystem
    #target_system = 0   # 0 --> broadcast to everyone
    lattitude = int(lat)
    longitude = int(lon)
    altitude = int(alt)

    msg = MAV_APM.MAVLink_set_gps_global_origin_message(
            target_system,
            lattitude, 
            longitude,
            altitude)

    send_message(msg, mav, pub)

def set_home_position(mav, pub):
    """
    Send a mavlink SET_HOME_POSITION message, which should allow
    us to use local position information without a GPS
    """
    target_system = mav.srcSystem
    #target_system = 0  # broadcast to everyone

    lattitude = int(lat)
    longitude = int(lon)
    altitude = int(alt)
    
    x = 0
    y = 0
    z = 0
    q = [1, 0, 0, 0]   # w x y z

    approach_x = 0
    approach_y = 0
    approach_z = 1

    msg = MAV_APM.MAVLink_set_home_position_message(
            target_system,
            lattitude,
            longitude,
            altitude,
            x,
            y,
            z,
            q,
            approach_x,
            approach_y,
            approach_z)

    send_message(msg, mav, pub)

def mav_callback(msg):
    a = 1
    #print("read mavlink!!")
    #print(msg)

    #lat = lat + 1000
    #set_global_origin(mav, mavlink_pub)


def vp_camera_callback(pose: PoseStamped):
    #print(landing_pub.name)
    if vp_status is None:
        return

    if vp_status.data == 0:
        print("vp error")
        return

    #print(vp_status)
    a = mavros_msgs.msg.LandingTarget()
    a.pose = pose.pose
    a.type = 2 # VISION_FIDUCIAL
    a.frame = 12 #BODY FRD
    a.target_num = 0
    a.distance = math.sqrt(a.pose.position.x**2 + a.pose.position.y**2 + a.pose.position.z**2)
    a.header = pose.header

    xx = a.pose.position.x
    yy = a.pose.position.y

    a.pose.position.x = xx
    a.pose.position.y = yy * -1
    a.pose.position.z = a.pose.position.z * -1

    landing_pub.publish(a)
    
    sens = Range()
    sens.header = a.header
    sens.range = a.pose.position.z * -1
    sens.min_range = 0
    sens.max_range = 10
    sens.field_of_view = 1
    rangefinder.publish(sens)

def vp_status_callback(status : Int8):
    global vp_status
    vp_status = status

if __name__=="__main__":
    try:
        node = rospy.init_node("origin_publisher")
        my_publisher = rospy.Publisher("/origin_publisher/my_topic_test", PoseStamped, queue_size=20)
        mavlink_pub = rospy.Publisher("/mavlink/to", Mavlink, queue_size=20)
        rospy.Subscriber("mavlink/from", Mavlink, mav_callback)
        rospy.Subscriber("vision/pose", PoseStamped, vp_camera_callback)
        rospy.Subscriber("vision/status", Int8, vp_status_callback)
        landing_pub = rospy.Publisher("mavros/landing_target/raw", LandingTarget)
        rangefinder = rospy.Publisher("mavros/distance_sensor/rangefinder_sub", Range)
        status_text_pub = rospy.Publisher("mavros/statustext/send", StatusText, queue_size=20)

        mav_pub = mavlink_pub
        # Set up mavlink instance
        f = fifo()
        mav = MAV_APM.MAVLink(f, srcSystem=2, srcComponent=1)

        # wait to initialize
        while mavlink_pub.get_num_connections() <= 0:
            pass

        while status_text_pub.get_num_connections() <= 0:
            pass

        print("start")
        text = StatusText()
        text.text = "RPI Start"
        text.severity = 4
        text.header.stamp = rospy.Time.now()
        text.header.frame_id
        status_text_pub.publish(text)

        while not rospy.is_shutdown():
            pose = PoseStamped()
            pose.pose.position.z = 10
            my_publisher.publish(pose)
            rospy.sleep(2)

    except rospy.ROSInterruptException:
        pass

