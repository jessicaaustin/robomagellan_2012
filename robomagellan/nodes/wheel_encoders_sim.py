#!/usr/bin/env python

#
# This node provides /wheel_odom with covariance.
# In simulation, the stage node provides odometry, but there is no
# covariance, which is required by robot_pose_ekf
#
# TODO the right way to fix this is to just provide a patch for
#      the stage package...
# 

import roslib; roslib.load_manifest('robomagellan')
import rospy

from nav_msgs.msg import Odometry

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


if __name__ == '__main__':
    rospy.init_node('wheel_encoders')
    rospy.sleep(3)  
    rospy.loginfo("Initializing wheel_encoders in sim mode")

    wheel_odom_pub = rospy.Publisher('/wheel_odom', Odometry)
    rospy.Subscriber('/wheel_odom_stage', Odometry, setup_wheel_odom_stage_callback(wheel_odom_pub))

    rospy.spin()
            
