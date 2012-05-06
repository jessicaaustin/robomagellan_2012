#!/usr/bin/env python

"""
ImuPublisher.py

Author: Bill Mania <bill@manialabs.us>

"""

import roslib; roslib.load_manifest('robomagellan')
import rospy

import sys

from math import pi
from IMU import IMU
from sensor_msgs import Imu as ImuMessage

def setupVisualizer():
    global visualizer

    visualizer = tf.TransformBroadcaster()
    visualizerRotation = tf.transformations.quaternion_from_euler(
        0,
        0,
        0
        )
    visualizer.sendTransform(
        visualizerTranslation,
        visualizerRotation,
        rospy.Time.now(),
        visualizerFrameName,
        visualizerFrameParent
        )

    return

def visualize():
    imu = IMU('/dev/ttyUSB0')

        try:
            visualizer.sendTransform(
                visualizerTranslation,
                visualizerRotation,
                rospy.Time.now(),
                visualizerFrameName,
                visualizerFrameParent
                )
        except:
            rospy.logwarn('sendTransform exception')
            rospy.logwarn(sys.exc_info()[0])
            rospy.logwarn(sys.exc_info()[1])

        rospy.sleep(0.5)

if __name__ == '__main__':
    rospy.init_node('ImuPublisher', log_level = rospy.DEBUG)

	imu = IMU('/dev/ttyUSB0')
	imuPublisher = rospy.Publisher('imu_data', ImuMessage)

	imuMessage = ImuMessage()
	imuMessage.header.frame_id = 'imu'

	imuMessage.orientation.x = 0.0
	imuMessage.orientation.y = 0.0
	imuMessage.orientation.z = 0.0
	imuMessage.orientation.w = 0.0

    while not rospy.is_shutdown():
        roll, pitch, yaw = imu.getOrientation()
        if roll < 0.0:
            roll = 360.0 + roll
        if pitch < 0.0:
            pitch = 360.0 + pitch
        if yaw < 0.0:
            yaw = 360.0 + yaw
        rospy.logdebug('orientation roll %f, pitch %f, yaw %f' % (roll, pitch, yaw))

        try:
            imuMessage.orientation = tf.transformations.quaternion_from_euler(
                roll / ( 2 * pi),
                pitch / ( 2 * pi),
                yaw / ( 2 * pi)
                )
        except:
            rospy.logwarn('Quaternion setup exception')
            rospy.logwarn(sys.exc_info()[0])
            rospy.logwarn(sys.exc_info()[1])
            continue
    
		imuMessage.header.stamp = rospy.Time.now()
		imuPublisher.publish(imuMessage)

