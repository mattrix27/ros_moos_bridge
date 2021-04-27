#!/usr/bin/env python

import sys
import rospy
from nmea_msgs.msg import Sentence
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped, Vector3Stamped
import tf
import math

pub = rospy.Publisher('nmea_to_send', Sentence, queue_size=10)
heading = 0.0
lat = 0.0
lon = 0.0
time = 0.0
speed = 0.0

def translate_to_nmea_cb(gps_msg):
  lat = gps_msg.latitude
  lon = gps_msg.longitude
  time = gps_msg.header.stamp
  
  #print ("gps time {} \n".format(time) )
  #quaternion = (
  #  odom_msg.pose.pose.orientation.x,
  #  odom_msg.pose.pose.orientation.y,
  #  odom_msg.pose.pose.orientation.z,
  #  odom_msg.pose.pose.orientation.w)
  #euler = tf.transformations.euler_from_quaternion(quaternion)
  #roll = euler[0]
  #pitch = euler[1]
  #yaw = euler[2]
  
  #heading = -(yaw*180.0/math.pi)+90.0;
  
  #speed_x = odom_msg.twist.twist.linear.x
  #speed_y = odom_msg.twist.twist.linear.y
  #speed = math.sqrt(speed_x**2 + speed_y**2)

  #nmea_sentence = "$PYGPS,%s,%f,%f,%f,%f*00\n" % (time, lat, lon, heading, speed)
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
  # *************** Heading is still a bit off not sure why here. Need to assess further. 
  #print ("SBG IMU \n", heading)
  
def xsens_speed_cb(msg):
  global speed
  speed = msg.vector.x
  #print("Xsens speed: {}".format(speed))

def sbg_speed_cb(msg):
  global speed
  speed = msg.twist.linear.x
  #print("SBG speed: {}".format(speed))

def test_bridge(msg):
  nmea_sentence = "$PYGPS,%s,%s\n" % (rospy.get_time(), msg.data) 

  nmea_msg = Sentence()
  nmea_msg.header.stamp = rospy.Time.now()
  nmea_msg.header.frame_id = "odom"
  nmea_msg.sentence = nmea_sentence
  pub.publish(nmea_msg)

def gps_to_nmea():
  rospy.init_node('ekf_to_nmea')
  rate = rospy.Rate(100)
  #rospy.Subscriber('/filter/gnss', NavSatFix, translate_to_nmea_cb)
  #rospy.Subscriber('/imu/data', Imu, xsens_imu_cb)
  #rospy.Subscriber('/filter/velocity', Vector3Stamped, xsens_speed_cb)
  rospy.Subscriber('/linear_velocity', TwistStamped, sbg_speed_cb)
  rospy.Subscriber('/sensor/gps/fix', NavSatFix, translate_to_nmea_cb)
  rospy.Subscriber('/imu/data/ned', Imu, imu_cb)
  rospy.Subscriber('/relative_pos', String, test_bridge)

  while not rospy.is_shutdown():
    rospy.spin()    
    
   
try:
  gps_to_nmea()
except rospy.ROSInterruptException:
  pass
