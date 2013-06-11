#!/usr/bin/env python

"""

baseController

- subscribes to the lmotor and rmotor topics
- uses the values received to drive each motor

listens to:
 /lmotor
 /rmotor

publishes to:

The message on the motor topic is std_msgs/Float32, whose
data element is interpreted as the number of meters per
second at which the wheel should be rotated. The sign of
that value is interpreted as the direction to rotate
the wheel.

"""

import roslib; roslib.load_manifest('robomagellan')
import rospy
from std_msgs.msg import Float32

from PhidgetMotorController import PhidgetMotorController

motorController = None

leftVelocity = 0.0
rightVelocity = 0.0

maxMetersPerSecond = 0.0
minMetersPerSecond = 0.0

def handleRightMotorMessage(rightMotorMessage):
    global rightVelocity

    if -(minMetersPerSecond) < rightMotorMessage.data < minMetersPerSecond:
        rightVelocity = 0
        return
    elif rightMotorMessage.data >= minMetersPerSecond:
        rightVelocity = int(rightMotorMessage.data / maxMetersPerSecond * 100)
        if rightVelocity > 100:
            rightVelocity = 100
    else:
        rightVelocity = int(rightMotorMessage.data / maxMetersPerSecond * 100)
        if rightVelocity < -100:
            rightVelocity = -100

    rightVelocity = -(rightVelocity)

    return

def handleLeftMotorMessage(leftMotorMessage):
    global leftVelocity

    if -(minMetersPerSecond) < leftMotorMessage.data < minMetersPerSecond:
        leftVelocity = 0
        return
    elif leftMotorMessage.data >= minMetersPerSecond:
        leftVelocity = int(leftMotorMessage.data / maxMetersPerSecond * 100)
        if leftVelocity > 100:
            leftVelocity = 100
    else:
        leftVelocity = int(leftMotorMessage.data / maxMetersPerSecond * 100)
        if leftVelocity < -100:
            leftVelocity = -100

    return

if __name__ == '__main__':
    rospy.init_node(
        name = 'diff_baseController',
        log_level = rospy.DEBUG
        )
    rospy.loginfo("Initializing diff_baseController node")

    motorController = PhidgetMotorController()
    motorController.setDefaultSpeed(75.0)

    maxMetersPerSecond = rospy.get_param("~maxMetersPerSecond", 0.5)
    minMetersPerSecond = rospy.get_param("~minMetersPerSecond", 0.1)
    rospy.loginfo("Max m/s: %0.3f, Min m/s: %0.3f" % (maxMetersPerSecond, minMetersPerSecond))

    rospy.Subscriber('lmotor', Float32, handleLeftMotorMessage)
    rospy.Subscriber('rmotor', Float32, handleRightMotorMessage)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        motorController.motorsDirect(
            leftVelocity,
            rightVelocity
            )
        rate.sleep()

    motorController.motorsDirect(0.0, 0.0)

