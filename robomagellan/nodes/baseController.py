#!/usr/bin/env python

"""

baseController

- subscribes to the cmd_vel topic to recieve Twist messages
- uses the parameters in the Twist message to translate and
  rotate the rover base

listens to:
 /cmd_vel

publishes to:

"""

import roslib; roslib.load_manifest('robomagellan')
import rospy

from geometry_msgs.msg import Twist
from PhidgetMotorController import PhidgetMotorController

motorController = None

lastCmdVelTime = None

def handleTwistMessage(twistMessage):
    global lastCmdVelTime

    lastCmdVelTime = rospy.Time.now().to_sec()

    rospy.logdebug("translationX: %f, rotationZ: %f" % (
        twistMessage.linear.x,
        twistMessage.angular.z
        ))

    motorController.move(
        twistMessage.linear.x,
        twistMessage.angular.z
        )

    return

def check_for_outdated_cmd_vel():
    global lastCmdVelTime

    now = rospy.Time.now().to_sec()
    latency = now - lastCmdVelTime
    if latency > 2.0:
        rospy.logdebug("cmd_vel command is %.2f seconds old! Stopping!" % latency)
        motorController.move(0.0, 0.0)


if __name__ == '__main__':
    rospy.init_node('base_controller')
    rospy.loginfo("Initializing base_controller node")

    motorController = PhidgetMotorController()
    motorController.setDefaultSpeed(75.0)

    lastCmdVelTime = rospy.Time.now().to_sec()

    rospy.Subscriber('cmd_vel', Twist, handleTwistMessage)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        check_for_outdated_cmd_vel()
        rate.sleep()

    motorController.move(0.0, 0.0)

