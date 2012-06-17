#!/usr/bin/env python

"""Phidgets 8/8/8 with two infrared rangefinders
and a battery voltage sensor

"""

import roslib; roslib.load_manifest('robomagellan')
import rospy

from Phidgets.Devices.InterfaceKit import InterfaceKit
from Phidgets.PhidgetException import PhidgetException
from sensor_msgs.msg import Range
from math import pi

class PhidgetInterface:

    def __init__(self):

        rospy.loginfo("Initializing PhidgetInterface")

        self.batteryVoltage = 0
        self.forwardRange = 1
        self.aftRange = 2

        self.interfaceKit = InterfaceKit()
        
        self.forwardRangeMessage = Range()
        self.forwardRangeMessage.header.frame_id = 'infrared_forward'
        self.forwardRangeMessage.radiation_type = Range.INFRARED
        self.forwardRangeMessage.field_of_view = 0.018
        self.forwardRangeMessage.min_range = 0.2
        self.forwardRangeMessage.max_range = 0.8
        self.forwardRangeMessage.range = 0.0

        self.aftRangeMessage = Range()
        self.aftRangeMessage.header.frame_id = 'infrared_aft'
        self.aftRangeMessage.radiation_type = Range.INFRARED
        self.aftRangeMessage.field_of_view = 0.018
        self.aftRangeMessage.min_range = 0.2
        self.aftRangeMessage.max_range = 0.8
        self.aftRangeMessage.range = 0.0

        self.sensorPublisher = rospy.Publisher('bogies', Range)

        self.interfaceKit.setOnAttachHandler(self.interfaceKitAttached)
        self.interfaceKit.setOnDetachHandler(self.interfaceKitDetached)
        self.interfaceKit.setOnErrorhandler(self.interfaceKitError)
        self.interfaceKit.setOnInputChangeHandler(self.interfaceKitInputChanged)
        self.interfaceKit.setOnOutputChangeHandler(self.interfaceKitOutputChanged)
        self.interfaceKit.setOnSensorChangeHandler(self.interfaceKitSensorChanged)

        try:
            self.interfaceKit.openPhidget()

        except PhidgetException, e:
            rospy.logerror("openPhidget() failed")
            rospy.logerror("code: %d" % e.code)
            rospy.logerror("message", e.message)

            raise

        try:
            self.interfaceKit.waitForAttach(10000)

        except PhidgetException, e:
            rospy.logerror("waitForAttach() failed")
            rospy.logerror("code: %d" % e.code)
            rospy.logerror("message", e.message)
    
            raise

	rospy.loginfo("InterfaceKit initialized")

        return

    def updateSensors(self):
        """Read the current value from each infrared sensor and update
           the Range messages for each.
        """

        self.forwardRangeMessage.header.stamp = rospy.Time.now()
        forwardRangeValue = self.interfaceKit.getSensorValue(self.forwardRange)
	rospy.logdebug("Forward range value %d" % (forwardRangeValue))
        self.forwardRangeMessage.range = int((0.0031912 * (forwardRangeValue ** 2)) - (1.3889 * forwardRangeValue) + 170.30)

        self.aftRangeMessage.header.stamp = rospy.Time.now()
        aftRangeValue = self.interfaceKit.getSensorValue(self.aftRange)
	rospy.logdebug("Aft range value %d" % (aftRangeValue))
        self.aftRangeMessage.range = int((0.0031912 * (aftRangeValue ** 2)) - (1.3889 * aftRangeValue) + 170.30)

        return

    def interfaceKitAttached(self, e):
        return

    def interfaceKitDetached(self, e):
        rospy.loginfo('encoderDetached() called')
        return
    
    def interfaceKitError(self, e):
        rospy.loginfo('encoderError() called')
        return
    
    def interfaceKitInputChanged(self, e):
        return
    
    def interfaceKitOutputChanged(self, e):
        return
    
    def interfaceKitSensorChanged(self, e):
        return
    
if __name__ == "__main__":
    rospy.init_node('short_range_and_battery', log_level = rospy.DEBUG)
    rospy.sleep(3.0)
    rospy.loginfo("Initializing short_range_and_battery.py")

    interfaceKit = PhidgetInterface()

    #
    # robot_pose_ekf uses a default update frequency of 30 Hz
    #
    consistentFrequency = rospy.Rate(30)
    while not rospy.is_shutdown():
        interfaceKit.updateSensors()
        interfaceKit.sensorPublisher.publish(interfaceKit.forwardRangeMessage)
        interfaceKit.sensorPublisher.publish(interfaceKit.aftRangeMessage)
        consistentFrequency.sleep()

