#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
import serial
import time
import socket
import pymap3d as pm
import threading
import queue
import os

record_path = os.path.join(os.getcwd(), "platsike.txt")

gps_port = '/dev/ttyACM0'
baudrate = 9600
try:
  ser = serial.Serial(gps_port, baudrate, timeout=0.5)
except serial.SerialException as e:
  print("Error:", e)

class GpsPublisher(object):
  def __init__(self):
    self.lat_diff = 0.0
    self.lon_diff = 0.0
    self.alt_diff = 0.0
    self.node = rospy.init_node('gps_publisher')

    self.publisher_raw = rospy.Publisher('gps_raw_data', NavSatFix, queue_size=10)
    self.publisher_enu = rospy.Publisher('gps_enu', PoseStamped, queue_size=10)

    timer_period = 0.05
    self.timer_raw = rospy.Timer(rospy.Duration(timer_period), self.gps_callback)

    self.lat0 = 58.3428685594
    self.lon0 = 25.5692475361
    self.alt0 = 91.357

    self.data_queue = queue.Queue()

    class SerialReader(threading.Thread):
      def __init__(self, queue, ser):
        super(SerialReader, self).__init__()
        self.queue = queue
        self.ser = ser

      def run(self):
        while True:
          line = self.ser.readline().decode('utf-8')
          self.queue.put(line)
    serial_reader = SerialReader(self.data_queue, ser)
    serial_reader.start()
    
    self.status_fix_thread = threading.Thread(target=self.status_fix_loop)
    self.status_fix_thread.daemon = True
    self.status_fix_thread.start()

    self.gps_base_tcp = self.gps_base_tcp
    self.get_logger = self.get_logger

  def gps_base_tcp(self):
    s = socket.socket()
    port = 8002
    s.connect(('213.168.5.170', port))
    return s.recv(1024)

  def get_logger(self):
    return rospy.get_logger()

  def status_fix(self):
    try:
      socket_info = self.gps_base_tcp()
      ser.write(socket_info)
    except Exception as e:
      self.get_logger().error(f"Error writing to u-blox device: {e}")

  def status_fix_loop(self):
    while True:
      self.status_fix()
      print(f"Status Fix on the GPS")
      time.sleep(1)

  def transform_to_enu(self, lat, lon, alt):
    x, y, z = pm.geodetic2enu(lat, lon, alt, self.lat0, self.lon0, self.alt0)
    return x, y, z

  def gps_callback(self, event):
    line = self.data_queue.get()
    print(f"Received line: {line}")
    line_split = line.split(",")
    print(f"Line split: {line_split}")
    if len(line_split) < 10:
      print("Skipping this line, not enough elements")
      return
    msg = NavSatFix()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "gps"
    try:
      lat_degrees = float(line_split[2][:2])
      lat_minutes = float(line_split[2][2:]) / 60
      lat = lat_degrees + lat_minutes
    except ValueError:
      lat = 0.0
    try:
      lon_degrees = float(line_split[4][:3])
      lon_minutes = float(line_split[4][3:]) / 60
      lon = lon_degrees + lon_minutes
    except ValueError:
      lon = 0.0
    if len(line_split) >= 10 and line_split[9].strip()!= '':
      msg.altitude = float(line_split[9])
    else:
      print("Skipping this line, not enough elements or invalid value in position 10")
      return
    alt = msg.altitude

    msg.latitude = lat
    msg.longitude = lon
    msg.altitude = alt
    self.publisher_raw.publish(msg)
    x, y, z = self.transform_to_enu(lat, lon, alt)
    enu_pose = PoseStamped()
    enu_pose.header.stamp = rospy.Time.now()
    enu_pose.header.frame_id = "enu"
    enu_pose.pose.position.x = x
    enu_pose.pose.position.y = y
    enu_pose.pose.position.z = z
    self.publisher_enu.publish(enu_pose)
    #with open(record_path, 'a') as f:
      #f.write("{}, {}, {}\n".format(x, y, z))

    #f = open("platsike.txt", "r")
    #print(f.read())

if __name__ == '__main__':
  gps_publisher = GpsPublisher()
  rospy.spin()
