#!/usr/bin/env python

import roslib; roslib.load_manifest('robomagellan')
import rospy

import tf

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

def setup_odom_callback():
    def odom_callback(data):
        odom = data
        (x,y) = (odom.pose.pose.position.x, odom.pose.pose.position.y)
        yaw = tf.transformations.euler_from_quaternion(odom.pose.pose.orientation)[2]
        print "odom (x,y) = (%f, %f) / yaw = %f" % (x,y,yaw)
    return setup_odom_callback

def setup_wheel_odom_callback():
    def odom_callback(data):
        odom = data
        (x,y) = (odom.pose.pose.position.x, odom.pose.pose.position.y)
        yaw = tf.transformations.euler_from_quaternion(odom.pose.pose.orientation)[2]
        print "wheel_odom (x,y) = (%f, %f) / yaw = %f" % (x,y,yaw)
    return setup_odom_callback

def setup_imu_callback():
    def imu_callback(data):
        imu = data
        yaw = tf.transformations.euler_from_quaternion(imu.orientation)[2]
        print "imu / yaw = %f" % yaw
    return setup_odom_callback

if __name__ == '__main__':
    rospy.init_node('odometry_debug')
    rospy.Subscriber('wheel_odom', Odometry, setup_wheel_odom_callback())
    rospy.Subscriber('imu_data', Imu, setup_imu_callback())
    rospy.Subscriber('odom', Odometry, setup_odom_callback())

