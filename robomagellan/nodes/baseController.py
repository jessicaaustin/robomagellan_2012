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

from rospy.exceptions import ROSInitException
from geometry_msgs.msg import Twist
from PhidgetMotorController import PhidgetMotorController

motorController = None

def handleTwistMessage(twistMessage):

    rospy.logdebug("translationX: %f, rotationZ: %f" % (
        twistMessage.linear.x,
        twistMessage.angular.z
        ))

    motorController.move(
        twistMessage.linear.x,
        twistMessage.angular.z
        )

    return

if __name__ == '__main__':
    rospy.init_node('base_controller')
    rospy.loginfo("Initializing base_controller node")

    motorController = PhidgetMotorController()
    motorController.setDefaultSpeed(75.0)

    rospy.Subscriber('cmd_vel', Twist, handleTwistMessage)

    while not rospy.is_shutdown():
        rospy.spin()

    motorController.move(0.0, 0.0)

