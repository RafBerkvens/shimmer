#!/usr/bin/env python

#  write_max_accel finds the max and min values of the accelerometer.
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

import yaml
import rospy
import sensor_msgs.msg
import geometry_msgs.msg

accel_x = []
accel_y = []
accel_z = []

def save():
  x_max = max(accel_x)
  x_min = min(accel_x)
  y_max = max(accel_y)
  y_min = min(accel_y)
  z_max = max(accel_z)
  z_min = min(accel_z)

  shimmer_minmax_data = dict()
  shimmer_minmax_data['accel_x_max'] = x_max
  shimmer_minmax_data['accel_x_min'] = x_min
  shimmer_minmax_data['accel_y_max'] = y_max
  shimmer_minmax_data['accel_y_min'] = y_min
  shimmer_minmax_data['accel_z_max'] = z_max
  shimmer_minmax_data['accel_z_min'] = z_min

  stream = file('shimmer_minmax_data.yaml', 'w')
  yaml.dump(shimmer_minmax_data, stream)
  
def imu_callback(msg):
  accel_x.append(msg.linear_acceleration.x)
  accel_y.append(msg.linear_acceleration.y)
  accel_z.append(msg.linear_acceleration.z)

if __name__ == '__main__':
  rospy.init_node('write_max_accel')
  rospy.Subscriber('/shimmer/imu', sensor_msgs.msg.Imu, imu_callback)

  print "Shake Shimmer hard in all axes' directions, then terminate program."

  rospy.on_shutdown(save)

  rospy.spin()
