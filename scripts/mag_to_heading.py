#!/usr/bin/env python
import rospy
import sensor_msgs
import tf
from math import pi, atan2, sqrt


def callback(data):
    yaw = -1.0 * atan2(float(data.magnetic_field.y),float(data.magnetic_field.x))
    print str(data.header.stamp.to_sec()) + ', ' + str(yaw)


def listener():
    rospy.init_node('mag_to_heading')
    rospy.Subscriber("/shimmer/mag", sensor_msgs.msg.MagneticField, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()