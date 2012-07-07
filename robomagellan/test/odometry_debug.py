#!/usr/bin/env python

import roslib; roslib.load_manifest('robomagellan')
import rospy

import tf

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

odom = (0, 0, 0)
wheel_odom = (0, 0, 0)
imu = (0, 0, 0)

def print_results():
    print odom, wheel_odom, imu

def odom_callback(data):
    odom = data
    (x,y) = (odom.pose.pose.position.x, odom.pose.pose.position.y)
    o = odom.pose.pose.orientation
    yaw = tf.transformations.euler_from_quaternion((o.x, o.y, o.z, o.w))[2]
    odom = (x,y,yaw)
    print_results()

def wheel_odom_callback(data):
    odom = data
    (x,y) = (odom.pose.pose.position.x, odom.pose.pose.position.y)
    o = odom.pose.pose.orientation
    yaw = tf.transformations.euler_from_quaternion((o.x, o.y, o.z, o.w))[2]
    wheel_odom = (x,y,yaw)
    print_results()

def imu_callback(data):
    imu = data
    o = imu.orientation
    yaw = tf.transformations.euler_from_quaternion((o.x, o.y, o.z, o.w))[2]
    imu = (0,0,yaw)
    print_results()

if __name__ == '__main__':
    rospy.init_node('odometry_debug')
    rospy.Subscriber('wheel_odom', Odometry, wheel_odom_callback)
    rospy.Subscriber('imu_data', Imu, imu_callback)
    rospy.Subscriber('odom', Odometry, odom_callback)

    rospy.spin()
