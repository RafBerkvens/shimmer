#!/usr/bin/env python

#  shimmer_pub_all publishes all shimmer imu data.
#  Copyright (C) 2013  Rafael Berkvens rafael.berkvens@uantwerpen.be
#
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program.  If not, see <http://www.gnu.org/licenses/>.

from math import pi, atan2

import sys, struct, array
import bluetooth
import rospy
import sensor_msgs.msg
import geometry_msgs.msg

def wait_for_ack():
   ddata = ""
   ack = struct.pack('B', 0xff)
   while ddata != ack:
      ddata = sock.recv(1)
   return
 
def read_accel_cal(sock):
  # send get accel calibration command
  sock.send(struct.pack('B', 0x13))
  
  # read the acknowledgement
  wait_for_ack()
  print "Acknowledgement received to get accel calibration command"
  
  # wait for calibration response 
  ddata = ""
  response = struct.pack('B', 0x12) 
  while ddata != response:
     ddata = sock.recv(1)
  print "Accel calibration response:"
  
  # read incoming data   
  ddata = ""
  numbytes = 0
  framesize = 21 
  
  while numbytes < framesize:
     ddata += sock.recv(framesize)
     numbytes = len(ddata)
  
  data = ddata[0:framesize]
  ddata = ddata[framesize:]
  numbytes = len(ddata)
  
  
  print "Raw packet received from shimmer:",
  print ",".join("0x{:02x}".format(ord(c)) for c in data)
  print
  
  (Xoffset, Yoffset, Zoffset, Xsensitivity, Ysensitivity, Zsensitivity, align0, align1, align2, align3, align4, align5, align6, align7, align8) = struct.unpack('>hhhhhhbbbbbbbbb', data);
  print "Offset Vector (ba) | Sensitivity Matrix (Ka) | Alignment Matrix (Ra)"
  print "              %4d |    %4d       0       0 |" % (Xoffset, Xsensitivity),
  print ' {: .2f}  {: .2f}  {: .2f}'.format(float(align0)/100, float(align1)/100, float(align2)/100)
  print "              %4d |       0    %4d       0 |" % (Yoffset, Ysensitivity),
  print ' {: .2f}  {: .2f}  {: .2f}'.format(float(align3)/100, float(align4)/100, float(align5)/100)
  print "              %4d |       0       0    %4d |" % (Zoffset, Zsensitivity),
  print ' {: .2f}  {: .2f}  {: .2f}'.format(float(align6)/100, float(align7)/100, float(align8)/100)
 
def read_gyro_cal(sock):
  # send get gyro calibration command
  sock.send(struct.pack('B', 0x16))
  
  # read the acknowledgement
  wait_for_ack()
  print "Acknowledgement received to get gyro calibration command"
  
  # wait for calibration response 
  ddata = ""
  response = struct.pack('B', 0x15) 
  while ddata != response:
     ddata = sock.recv(1)
  print "Gyro calibration response:"
  
  # read incoming data   
  ddata = ""
  numbytes = 0
  framesize = 21 
  
  while numbytes < framesize:
     ddata += sock.recv(framesize)
     numbytes = len(ddata)
  
  data = ddata[0:framesize]
  ddata = ddata[framesize:]
  numbytes = len(ddata)
  
  
  print "Raw packet received from shimmer:",
  print ",".join("0x{:02x}".format(ord(c)) for c in data)
  print
  
  (Xoffset, Yoffset, Zoffset, Xsensitivity, Ysensitivity, Zsensitivity, align0, align1, align2, align3, align4, align5, align6, align7, align8) = struct.unpack('>hhhhhhbbbbbbbbb', data);
  print "Offset Vector (bg) | Sensitivity Matrix (Kg) | Alignment Matrix (Rg)"
  print "              %4d |    %4d       0       0 |" % (Xoffset, Xsensitivity),
  print ' {: .2f}  {: .2f}  {: .2f}'.format(float(align0)/100, float(align1)/100, float(align2)/100)
  print "              %4d |       0    %4d       0 |" % (Yoffset, Ysensitivity),
  print ' {: .2f}  {: .2f}  {: .2f}'.format(float(align3)/100, float(align4)/100, float(align5)/100)
  print "              %4d |       0       0    %4d |" % (Zoffset, Zsensitivity),
  print ' {: .2f}  {: .2f}  {: .2f}'.format(float(align6)/100, float(align7)/100, float(align8)/100)
 
def read_mag_cal(sock):
  # send get mag calibration command
  sock.send(struct.pack('B', 0x19))
  
  # read the acknowledgement
  wait_for_ack()
  print "Acknowledgement received to get mag calibration command"
  
  # wait for calibration response 
  ddata = ""
  response = struct.pack('B', 0x18) 
  while ddata != response:
     ddata = sock.recv(1)
  print "Mag calibration response:"
  
  # read incoming data   
  ddata = ""
  numbytes = 0
  framesize = 21 
  
  while numbytes < framesize:
     ddata += sock.recv(framesize)
     numbytes = len(ddata)
  
  data = ddata[0:framesize]
  ddata = ddata[framesize:]
  numbytes = len(ddata)
  
  
  print "Raw packet received from shimmer:",
  print ",".join("0x{:02x}".format(ord(c)) for c in data)
  print
  
  (Xoffset, Yoffset, Zoffset, Xsensitivity, Ysensitivity, Zsensitivity, align0, align1, align2, align3, align4, align5, align6, align7, align8) = struct.unpack('>hhhhhhbbbbbbbbb', data);
  print "Offset Vector (bm) | Sensitivity Matrix (Km) | Alignment Matrix (Rm)"
  print "              %4d |    %4d       0       0 |" % (Xoffset, Xsensitivity),
  print ' {: .2f}  {: .2f}  {: .2f}'.format(float(align0)/100, float(align1)/100, float(align2)/100)
  print "              %4d |       0    %4d       0 |" % (Yoffset, Ysensitivity),
  print ' {: .2f}  {: .2f}  {: .2f}'.format(float(align3)/100, float(align4)/100, float(align5)/100)
  print "              %4d |       0       0    %4d |" % (Zoffset, Zsensitivity),
  print ' {: .2f}  {: .2f}  {: .2f}'.format(float(align6)/100, float(align7)/100, float(align8)/100)

def read_data(sock):
  # read incoming data
  ddata = ""
  numbytes = 0
  framesize = 21  # Packet type (1), TimeStamp (2), 3xAccel (3x2), 3xGyro (3x2), 3xMag (3x2)
  while numbytes < framesize:
    ddata += sock.recv(framesize)
    numbytes = len(ddata)

    data = ddata[0:framesize]
    ddata = ddata[framesize:]
    # numbytes = len(ddata)

  packettype = struct.unpack('B', data[0:1])
  return struct.unpack('HHHHHHHhhh', data[1:framesize])

if __name__ == '__main__':
  rospy.init_node('shimmer_pub_all')
  imu_pub = rospy.Publisher('shimmer/imu', sensor_msgs.msg.Imu)
  imu = sensor_msgs.msg.Imu()
  imu.header.frame_id = "imu"
  imu.orientation.x = 0
  imu.orientation.y = 0
  imu.orientation.z = 0
  imu.orientation.w = 0
  imu.orientation_covariance = [0] * 9
  imu.orientation_covariance[0] = -1  # covariance unknown
  imu.angular_velocity_covariance = [0] * 9
  imu.angular_velocity_covariance[0] = -1  # covariance unknown
  imu.linear_acceleration_covariance = [0] * 9
  imu.linear_acceleration_covariance[0] = -1  # covariance unknown
  
  mag_pub = rospy.Publisher('shimmer/mag', sensor_msgs.msg.MagneticField)
  mag = sensor_msgs.msg.MagneticField()
  mag.header.frame_id = "imu"
  mag.magnetic_field_covariance = [0] * 9
  mag.magnetic_field_covariance[0] = -1  # covariance unknown

  bd_addr = "00:06:66:43:B7:B7"
  port = 1
  sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
  sock.connect((bd_addr, port))
  
  read_accel_cal(sock)
  read_gyro_cal(sock)
  read_mag_cal(sock)

  # send the set sensors command
  sock.send(struct.pack('BBB', 0x08, 0xE0, 0x00))  # all
  wait_for_ack()
  # send the set sampling rate command
  sock.send(struct.pack('BB', 0x05, 0x14))  # 51.2Hz
  wait_for_ack()
  # send start streaming command
  sock.send(struct.pack('B', 0x07))
  wait_for_ack()

  while not rospy.is_shutdown():
    (timestamp,
     imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z,
     imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z,
     mag.magnetic_field.x, mag.magnetic_field.y, mag.magnetic_field.z) = read_data(sock)
    time = rospy.Time.now()
    imu.header.stamp = time
    imu_pub.publish(imu)
    mag.header.stamp = time
    mag_pub.publish(mag)
    rospy.logdebug("published!")
    rospy.sleep(0.02)
    
    #print atan2(float(mag.magnetic_field.y),float(mag.magnetic_field.x))*180/pi
    #http://www.loveelectronics.co.uk/Tutorials/13/tilt-compensated-compass-arduino-tutorial

  # send stop streaming command
  sock.send(struct.pack('B', 0x20));
  wait_for_ack()
  # close the socket
  sock.close()
  print "\n"

