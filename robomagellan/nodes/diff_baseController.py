#!/usr/bin/env python

"""

baseController

- subscribes to the lmotor and rmotor topics
- uses the values received to drive each motor

listens to:
 /lmotor
 /rmotor

publishes to:

"""

import roslib; roslib.load_manifest('robomagellan')
import rospy
from std_msgs.msg import Float32

from PhidgetMotorController import PhidgetMotorController

motorController = None

leftVelocity = 0.0
rightVelocity = 0.0

def handleRightMotorMessage(rightMotorMessage):
    global rightVelocity

    rospy.logdebug("rightVelocity: %f" % (
        rightMotorMessage.data
        ))

    rightVelocity = -(rightMotorMessage.data)

    return

def handleLeftMotorMessage(leftMotorMessage):
    global leftVelocity

    rospy.logdebug("leftVelocity: %f" % (
        leftMotorMessage.data
        ))

    leftVelocity = leftMotorMessage.data

    return

if __name__ == '__main__':
    rospy.init_node(
        name = 'diff_baseController',
        log_level = rospy.DEBUG
        )
    rospy.loginfo("Initializing diff_baseController node")

    motorController = PhidgetMotorController()
    motorController.setDefaultSpeed(75.0)

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

