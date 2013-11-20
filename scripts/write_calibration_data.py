#!/usr/bin/env python

#  write_calibration_data reads from shimmer and writes to file.
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

import sys, struct, array
import bluetooth
import yaml
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

  # wait for calibration response
  ddata = ""
  response = struct.pack('B', 0x12)
  while ddata != response:
     ddata = sock.recv(1)

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

  return struct.unpack('>hhhhhhbbbbbbbbb', data)

def read_gyro_cal(sock):
  # send get gyro calibration command
  sock.send(struct.pack('B', 0x16))

  # read the acknowledgement
  wait_for_ack()

  # wait for calibration response
  ddata = ""
  response = struct.pack('B', 0x15)
  while ddata != response:
     ddata = sock.recv(1)

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

  return struct.unpack('>hhhhhhbbbbbbbbb', data)

def read_mag_cal(sock):
  # send get mag calibration command
  sock.send(struct.pack('B', 0x19))

  # read the acknowledgement
  wait_for_ack()

  # wait for calibration response
  ddata = ""
  response = struct.pack('B', 0x18)
  while ddata != response:
     ddata = sock.recv(1)

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

  return struct.unpack('>hhhhhhbbbbbbbbb', data)

if __name__ == '__main__':
  rospy.init_node('write_calibration_data')

  #bd_addr = "00:06:66:43:B7:B7"
  bd_addr = "00:06:66:43:A9:0E"
  port = 1
  sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
  sock.connect((bd_addr, port))

  shimmer_calibration_data = dict()

  (Xoffset, Yoffset, Zoffset,
   Xsensitivity, Ysensitivity, Zsensitivity,
   align0, align1, align2,
   align3, align4, align5,
   align6, align7, align8) = read_accel_cal(sock)
  shimmer_calibration_data['shimmer/accel'] = dict(offset_vector=[Xoffset, Yoffset, Zoffset],
                                                   sensitivity_matrix=[Xsensitivity, Ysensitivity, Zsensitivity],
                                                   alignment_matrix=[float(x) / 100 for x in
                                                                     [align0, align1, align2,
                                                                      align3, align4, align5,
                                                                      align6, align7, align8]])
  (Xoffset, Yoffset, Zoffset,
   Xsensitivity, Ysensitivity, Zsensitivity,
   align0, align1, align2,
   align3, align4, align5,
   align6, align7, align8) = read_gyro_cal(sock)
  shimmer_calibration_data['shimmer/gyro'] = dict(offset_vector=[Xoffset, Yoffset, Zoffset],
                                                  sensitivity_matrix=[float(x) / 100 for x in
                                                                      Xsensitivity, Ysensitivity, Zsensitivity],
                                                  alignment_matrix=[float(x) / 100 for x in
                                                                    [align0, align1, align2,
                                                                     align3, align4, align5,
                                                                     align6, align7, align8]])
  (Xoffset, Yoffset, Zoffset,
   Xsensitivity, Ysensitivity, Zsensitivity,
   align0, align1, align2,
   align3, align4, align5,
   align6, align7, align8) = read_mag_cal(sock)
  shimmer_calibration_data['shimmer/mag'] = dict(offset_vector=[Xoffset, Yoffset, Zoffset],
                                                 sensitivity_matrix=[Xsensitivity, Ysensitivity, Zsensitivity],
                                                 alignment_matrix=[float(x) / 100 for x in
                                                                   [align0, align1, align2,
                                                                    align3, align4, align5,
                                                                    align6, align7, align8]])
  stream = file('shimmer_calibration_data.yaml', 'w')
  yaml.dump(shimmer_calibration_data, stream)

  # close the socket
  sock.close()
