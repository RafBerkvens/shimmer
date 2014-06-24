#!/usr/bin/env python
import rospy
import geometry_msgs
import tf


def callback(data):
    quat = (data.pose.orientation.x,
            data.pose.orientation.y,
            data.pose.orientation.z,
            data.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quat)
    yaw = euler[2]
    print str(data.header.stamp.to_sec()) + ', ' + str(yaw)


def listener():
    rospy.init_node('quat_to_heading')
    rospy.Subscriber("heading", geometry_msgs.msg.PoseStamped, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()