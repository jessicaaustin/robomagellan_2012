#!/usr/bin/env python

"""

motors_calibration

Subscribe to the imu_data and odom topics and display
their values

"""

import roslib; roslib.load_manifest('robomagellan')
import rospy
import math

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf import transformations

currentOdomX = 0.0
currentOdomY = 0.0
currentWheelOdomX = 0.0
currentWheelOdomY = 0.0
currentOdomTheta = 0.0
currentWheelOdomTheta = 0.0
currentImuTheta = 0.0
currentTranslate = 0.0
currentRotate = 0.0
currentOdomTime = 0.0
currentWheelOdomTime = 0.0
currentImuTime = 0.0

def handleOdometryMessage(odometryMessage):
    global currentOdomX, currentOdomY, currentOdomTheta, currentOdomTime

    currentOdomTime = odometryMessage.header.stamp.to_sec()
    currentOdomX = odometryMessage.pose.pose.position.x
    currentOdomY = odometryMessage.pose.pose.position.y
    orientation = [
        odometryMessage.pose.pose.orientation.x,
        odometryMessage.pose.pose.orientation.y,
        odometryMessage.pose.pose.orientation.z,
        odometryMessage.pose.pose.orientation.w
        ]
    currentOdomTheta = transformations.euler_from_quaternion(orientation)[2]

    return

def handleWheelOdometryMessage(odometryMessage):
    global currentWheelOdomX, currentWheelOdomY, currentWheelOdomTheta, currentWheelOdomTime

    currentWheelOdomTime = odometryMessage.header.stamp.to_sec()
    currentWheelOdomX = odometryMessage.pose.pose.position.x
    currentWheelOdomY = odometryMessage.pose.pose.position.y
    orientation = [
        odometryMessage.pose.pose.orientation.x,
        odometryMessage.pose.pose.orientation.y,
        odometryMessage.pose.pose.orientation.z,
        odometryMessage.pose.pose.orientation.w
        ]
    currentWheelOdomTheta = transformations.euler_from_quaternion(orientation)[2]

    return


def handleImuMessage(imuMessage):
    global currentImuTheta, currentImuTime

    currentImuTime = imuMessage.header.stamp.to_sec()
    orientation = [
        imuMessage.orientation.x,
        imuMessage.orientation.y,
        imuMessage.orientation.z,
        imuMessage.orientation.w
        ]

    currentImuTheta = transformations.euler_from_quaternion(orientation)[2]

    return

def handleTwistMessage(twistMessage):
    global currentTranslate, currentRotate

    currentTranslate = twistMessage.linear.x
    currentRotate = twistMessage.angular.z

    return

def displayPoseAndTwist():
    print "T m/s: %5.2f, R d/s: %d | Encod: (%5.2f, %5.2f), EKF: (%5.2f, %5.2f) | Encod_theta: %3d, IMU_theta: %3d, EKF_theta: %3d" % (
        currentTranslate,
        currentRotate * 360 / (2 * math.pi),
        currentWheelOdomX,
        currentWheelOdomY,
        currentOdomX,
        currentOdomY,
        currentWheelOdomTheta  * 360 / (2 * math.pi),
        currentImuTheta  * 360 / (2 * math.pi),
        currentOdomTheta  * 360 / (2 * math.pi)
        )

    return

if __name__ == '__main__':
    rospy.init_node('motors_calibration')

    rospy.Subscriber('imu_data', Imu, handleImuMessage)
    rospy.Subscriber('wheel_odom', Odometry, handleWheelOdometryMessage)
    rospy.Subscriber('odom', Odometry, handleOdometryMessage)
    rospy.Subscriber('cmd_vel', Twist, handleTwistMessage)

    consistentFrequency = rospy.Rate(4)
    while not rospy.is_shutdown():
        displayPoseAndTwist()

        consistentFrequency.sleep()
