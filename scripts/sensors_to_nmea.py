#!/usr/bin/env python3

import sys
import rospy
from nmea_msgs.msg import Sentence
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped, Vector3Stamped, Point
from std_msgs.msg import UInt16
import tf
import math

pub = rospy.Publisher('nmea_to_send', Sentence, queue_size=10)
rc_pub = rospy.Publisher('/rc_command', UInt16, queue_size=10)
heading = 0.0
lat = 0.0
lon = 0.0
time = 0.0
speed = 0.0

def translate_to_nmea_cb(gps_msg):
  lat = gps_msg.latitude
  lon = gps_msg.longitude
  time = gps_msg.header.stamp
  
  nmea_sentence = "$PYGPS,%s,%f,%f,%f,%f\n" % (rospy.get_time(), lat, lon, heading, speed) 
  
  nmea_msg = Sentence()
  nmea_msg.header.stamp = time #rospy.Time.now()
  nmea_msg.header.frame_id = "odom"
  nmea_msg.sentence = nmea_sentence
  pub.publish(nmea_msg)


def xsens_imu_cb(message):
  global heading
  q = message.orientation
  euler = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[:3]
  degrees = [180/3.14*x for x in euler]
  heading = 90-degrees[2]
  
  print ("XSENS IMU \n", heading)


def imu_cb(message):
  global heading
  q = message.orientation
  euler = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[:3]
  degrees = [180/3.14*x for x in euler]
  heading = 90-math.fmod(euler[2], 360.0)  # 180-degrees[2]


def xsens_speed_cb(msg):
  global speed
  speed = msg.vector.x
  #print("Xsens speed: {}".format(speed))


def sbg_speed_cb(msg):
  global speed
  speed = msg.twist.linear.x
  #print("SBG speed: {}".format(speed))


def translate_pos_to_nmea_cb(msg):
  nmea_sentence = "POS,%s,%f,%f\n" % (rospy.get_time(), msg.x, msg.y)

  nmea_msg = Sentence()
  nmea_msg.header.stamp = rospy.Time.now()
  nmea_msg.header.frame_id = "odom"
  nmea_msg.sentence = nmea_sentence
  pub.publish(nmea_msg)


def rc_command_cb(msg):
  command = msg.sentence
  if "FLIP" in command:
    rc_pub.publish(3)
  elif "RESET" in command:
    rc_pub.publish(0)
  elif "START" in command:
    rc_pub.publish(1)


def sensors_to_nmea():
  rospy.init_node('sensors_to_nmea')
  rate = rospy.Rate(100)
  #rospy.Subscriber('/filter/gnss', NavSatFix, translate_to_nmea_cb)
  #rospy.Subscriber('/imu/data', Imu, xsens_imu_cb)
  #rospy.Subscriber('/filter/velocity', Vector3Stamped, xsens_speed_cb)
  rospy.Subscriber('/linear_velocity', TwistStamped, sbg_speed_cb)
  rospy.Subscriber('/sensor/gps/fix', NavSatFix, translate_to_nmea_cb)
  rospy.Subscriber('/imu/data/ned', Imu, imu_cb)
  rospy.Subscriber('/relative_pos', Point, translate_pos_to_nmea_cb)
  rospy.Subscriber('/nmea_received', Sentence, rc_command_cb)

  while not rospy.is_shutdown():
    rospy.spin()    
    
   
try:
  sensors_to_nmea()
except rospy.ROSInterruptException:
  pass
