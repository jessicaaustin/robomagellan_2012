#!/usr/bin/env python

"""ImuPublisher.py

Author: Bill Mania <bill@manialabs.us>

"""

import roslib; roslib.load_manifest('robomagellan')
import rospy

import sys

from math import pi
from IMU import IMU
from sensor_msgs.msg import Imu as ImuMessage
import tf


if __name__ == '__main__':
    rospy.init_node('ImuPublisher', log_level = rospy.INFO)

    rospy.sleep(2)
    rospy.loginfo("Initializing ImuPublisher node")

    imu = IMU('/dev/ttyUSB0')
    #
    # see the rename in the import above
    #
    imuPublisher = rospy.Publisher('imu_data', ImuMessage)

    imuMessage = ImuMessage()
    imuMessage.header.frame_id = 'imu'

    imuMessage.orientation.x = 0.0
    imuMessage.orientation.y = 0.0
    imuMessage.orientation.z = 0.0
    imuMessage.orientation.w = 0.0
    imuMessage.orientation_covariance = [1000.0, 0.0, 0.0,
                                         0.0, 1000.0, 0.0,
                                         0.0, 0.0, 1000.0]
    #
    # angular velocity
    #
    imuMessage.angular_velocity.x = 0.0
    imuMessage.angular_velocity.y = 0.0
    imuMessage.angular_velocity.z = 0.0
    imuMessage.angular_velocity_covariance = [1000.0, 0.0, 0.0,
                                              0.0, 1000.0, 0.0,
                                              0.0, 0.0, 1000.0]
    #
    # linear acceleration not available
    #
    imuMessage.linear_acceleration_covariance = [-1, 0.0, 0.0,
                                                 0.0, 0.0, 0.0,
                                                 0.0, 0.0, 0.0]

    yawOffset = None
    previousTimestamp = rospy.get_time()
    previousYaw = None

    while not rospy.is_shutdown():
        currentTimestamp = rospy.get_time()
        roll, pitch, yaw, exception = imu.getOrientation()
        if exception:
            rospy.logwarn("getOrientation(): %s" % (exception))
            continue

        deltaTime = currentTimestamp - previousTimestamp

#        roll = roll / 180 * pi
#        pitch = pitch / 180 * pi
        yaw = -1 * yaw / 180 * pi

        if not yawOffset:
            yawOffset = yaw

        # the robot starts off at yaw = 0, and rotates in deltas from this initial orientation
        yaw -= yawOffset

        if previousYaw:
            imuMessage.angular_velocity.z = (yaw - previousYaw) / deltaTime

        previousYaw = yaw
        previousTimestamp = currentTimestamp
            
        try:
            q = tf.transformations.quaternion_about_axis(yaw, (0, 0, 1))
            imuMessage.orientation.x = q[0]
            imuMessage.orientation.y = q[1]
            imuMessage.orientation.z = q[2]
            imuMessage.orientation.w = q[3]
        except:
            rospy.logwarn('Quaternion setup exception')
            rospy.logwarn(sys.exc_info()[0])
            rospy.logwarn(sys.exc_info()[1])
            continue
    
        imuMessage.header.stamp = rospy.Time.now()
        imuPublisher.publish(imuMessage)

