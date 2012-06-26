#!/usr/bin/env python

#
# This node augments the stage node to provide a more realistic simulation
#
# /wheel_odom:
#  In simulation, the stage node provides odometry, but there is no
#  covariance, which is required by robot_pose_ekf.
#  So this node takes in stage's /wheel_odom_stage, adds covariance, 
#  and re-publishes on the /wheel_odom topic.
#
# /base_scan
#  Takes in stage's /base_scan_stage message and augments it to
#  create a more noisy and realistic /base_scan topic.
#

import roslib; roslib.load_manifest('robomagellan')
import rospy

import random

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

def setup_wheel_odom_stage_callback(wheel_odom_pub):
    def wheel_odom_stage_callback(data):
        wheel_odom = data
        covariance = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                      0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
                      0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                      0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
                      0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        wheel_odom.pose.covariance = covariance
        wheel_odom.twist.covariance = covariance
        wheel_odom_pub.publish(wheel_odom)
    return wheel_odom_stage_callback

def setup_base_scan_stage_callback(base_scan_pub):
    def base_scan_stage_callback(data):
        base_scan = data
        updated_ranges = []
        for r in base_scan.ranges:
            # if it's at stage's max range, add some noise
            if r >= 3.0:
                updated_ranges.append(random.uniform(2.0, 3.6))
            else:
                updated_ranges.append(r)
        base_scan.ranges = updated_ranges
        base_scan_pub.publish(base_scan)
    return base_scan_stage_callback


if __name__ == '__main__':
    rospy.init_node('wheel_encoders')
    rospy.sleep(3)  
    rospy.loginfo("Initializing wheel_encoders in sim mode")

    wheel_odom_pub = rospy.Publisher('/wheel_odom', Odometry)
    rospy.Subscriber('/wheel_odom_stage', Odometry, setup_wheel_odom_stage_callback(wheel_odom_pub))

    base_scan_pub = rospy.Publisher('/base_scan', LaserScan)
    rospy.Subscriber('/base_scan_stage', LaserScan, setup_base_scan_stage_callback(base_scan_pub))

    rospy.spin()
            
