#!/usr/bin/env python

"""

motors_calibration

Subscribe to the imu_data and wheel_odom topics and display
their values

"""

import roslib; roslib.load_manifest('robomagellan')
import rospy
import math

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf import transformations

currentX = 0.0
currentY = 0.0
currentOdomTheta = 0.0
currentImuTheta = 0.0
currentTranslate = 0.0
currentRotate = 0.0

def handleOdometryMessage(odometryMessage):
    global currentX, currentY, currentOdomTheta

    currentX = odometryMessage.pose.pose.position.x
    currentY = odometryMessage.pose.pose.position.y
    orientation = [
        odometryMessage.pose.pose.orientation.x,
        odometryMessage.pose.pose.orientation.y,
        odometryMessage.pose.pose.orientation.z,
        odometryMessage.pose.pose.orientation.w
        ]
    currentOdomTheta = transformations.euler_from_quaternion(orientation)[2]

    return

def handleImuMessage(imuMessage):
    global currentImuTheta

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
    print "%d - T m/s: %5.2f, R d/s: %d X m: %5.2f, Y m: %5.2f, OdomTheta d: %3d, ImuTheta d: %3d" % (
        rospy.get_time(),
        currentTranslate,
        currentRotate * 360 / (2 * math.pi),
        currentX,
        currentY,
        currentOdomTheta  * 360 / (2 * math.pi),
        currentImuTheta  * 360 / (2 * math.pi)
        )

    return

if __name__ == '__main__':
    rospy.init_node('motors_calibration')

    rospy.Subscriber('imu_data', Imu, handleImuMessage)
    rospy.Subscriber('wheel_odom', Odometry, handleOdometryMessage)
    rospy.Subscriber('cmd_vel', Twist, handleTwistMessage)

    consistentFrequency = rospy.Rate(4)
    while not rospy.is_shutdown():
        displayPoseAndTwist()

        consistentFrequency.sleep()
