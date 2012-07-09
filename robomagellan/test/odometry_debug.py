#!/usr/bin/env python

import roslib; roslib.load_manifest('robomagellan')
import rospy

import tf

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

cmd_vel = [('cmd_vel_time', 'cmd_vel_x', 'cmd_vel_yaw')]
imu = [('imu_time','imu_yaw')]
wheel_odom = [('wheel_odom_time', 'wheel_odom_x', 'wheel_odom_y', 'wheel_odom_yaw')]
odom = [('odom_time', 'odom_x', 'odom_y', 'odom_yaw')]

def print_results():
    print cmd_vel
    print imu
    print wheel_odom
    print odom

def cmd_vel_callback(data):
    (x,yaw) = (data.linear.x, data.angular.z)
    cmd_vel.append((rospy.Time.now(), x, yaw))

def odom_callback(data):
    (x,y) = (data.pose.pose.position.x, data.pose.pose.position.y)
    o = data.pose.pose.orientation
    yaw = tf.transformations.euler_from_quaternion((o.x, o.y, o.z, o.w))[2]
    odom.append((data.header.stamp, x,y,yaw))

def wheel_odom_callback(data):
    (x,y) = (data.pose.pose.position.x, data.pose.pose.position.y)
    o = data.pose.pose.orientation
    yaw = tf.transformations.euler_from_quaternion((o.x, o.y, o.z, o.w))[2]
    wheel_odom.append((data.header.stamp,x,y,yaw))

def imu_callback(data):
    o = data.orientation
    yaw = tf.transformations.euler_from_quaternion((o.x, o.y, o.z, o.w))[2]
    imu.append((data.header.stamp,yaw))

if __name__ == '__main__':
    rospy.init_node('odometry_debug')
    rospy.Subscriber('wheel_odom', Odometry, wheel_odom_callback)
    rospy.Subscriber('imu_data', Imu, imu_callback)
    rospy.Subscriber('odom', Odometry, odom_callback)

    pub_cmd_vel = rospy.Publisher('cmd_vel', Twist)

    # move in a straight line forward for one second
    print "moving in a straight line forward at .5 m/s for 2 sec"
    t = Twist()
    t.x = 0.5
    for i in range(0,20):
        pub_cmd_vel.publish(t)
        rospy.sleep(.1)
    print_results()
    

