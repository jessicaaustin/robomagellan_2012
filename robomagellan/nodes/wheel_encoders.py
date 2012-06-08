#!/usr/bin/env python

"""Phidgets dual encoders

"""

import roslib; roslib.load_manifest('robomagellan')
import rospy

from ctypes import *
from Phidgets.Devices.Encoder import Encoder
from Phidgets.PhidgetException import PhidgetException
from nav_msgs.msg import Odometry

class PhidgetEncoders:

    def __init__(self):

        print "Initializing PhidgetEncoders"

        self.leftEncoder = 0 # forward is negative
        self.rightEncoder = 1 # forward is positive

        self.previousXPosition = 0
        self.previousYPosition = 0

        self.encoder = Encoder()
        self.odometryMessage = Odometry()

        self.encoder.setOnAttachHandler(self.encoderAttached)
        self.encoder.setOnDetachHandler(self.encoderDetached)
        self.encoder.setOnErrorhandler(self.encoderError)
        self.encoder.setOnInputChangeHandler(self.encoderInputChanged)
        self.encoder.setOnPositionChangeHandler(self.encoderPositionChange)

        try:
            rospy.loginfo('openPhidget()')
            print "openPhidget()"
            self.encoder.openPhidget()

        except PhidgetException, e:
            rospy.logerror("openPhidget() failed")
            rospy.logerror("code: %d" % e.code)
            rospy.logerror("message", e.message)

            raise

        try:
            rospy.loginfo('waitForAttach()')
            print "waitForAttach()"
            self.encoder.waitForAttach(10000)

        except PhidgetException, e:
            rospy.logerror("waitForAttach() failed")
            rospy.logerror("code: %d" % e.code)
            rospy.logerror("message", e.message)
    
            raise

        rospy.loginfo('Encoder device: %s' % (self.encoder.getDeviceName()))
        rospy.loginfo('        serial: %d' % (self.encoder.getSerialNum()))
        rospy.loginfo('        version: %d' % (self.encoder.getDeviceVersion()))
        rospy.loginfo('        count: %d' % (self.encoder.getEncoderCount()))
        rospy.loginfo('        input count: %d' % (self.encoder.getInputCount()))


        self.encoderPublisher = rospy.Publisher('wheel_odom', Odometry)

        print "Done initializing"

        return

    def encoderPositionChange(self, e):
        """Called each time the encoder reports a change to its position.
           This method will convert the encoder values to an updated
           position in the base_link frame and then publish an Odometry
           message to the wheel_odom topic.
        """
        rospy.logdebug('Encoder %i Change: %i Time: %i' % (
            e.index,
            e.positionChange,
            e.time
            )
            )

        self.odometryMessage.header.stamp = rospy.Time.now()
        deltaTSeconds = e.time * UNITS_PER_SECOND
        if e.index == self.leftEncoder:
                leftEncoderValue = -1 * e.positionChange
                rightEncoderValue = self.encoder.getPosition(self.rightEncoder)
        else:
                rightEncoderValue = e.positionChange
                leftEncoderValue = -1 * self.encoder.getPosition(self.leftEncoder)

        #
        # determine how far each wheel has traveled
        #
        leftWheelDistance = leftEncoderValue * LEFT_METERS_PER_PULSE
        rightWheelDistance = rightEncoderValue * RIGHT_METERS_PER_PULSE
        averagedDistance = (leftWheelDistance + rightWheelDistance) / 2.0
        angleOfTravel = (leftWheelDistance - rightWheelDistance) / METERS_BETWEEN_WHEELS


        #
        # determine the current Pose and Twist
        #

        #
        # stick them into self.odometryMessage
        #

        #
        # publish to wheel_odom
        #
        self.encoderPublisher(self.odometryMessage)

        return

    def encoderAttached(self, e):
        rospy.loginfo('encoderAttached() called')

        self.encoder.setPosition(
                self.leftEncoder,
                0
                )
        self.encoder.setPosition(
                self.rightEncoder,
                0
                )
        self.encoder.setEnabled(
                self.leftEncoder,
                True
                )
        self.encoder.setEnabled(
                self.rightEncoder,
                True
                )

        return
    
    def encoderDetached(self, e):
        rospy.loginfo('encoderDetached() called')
        return
    
    def encoderError(self, e):
        rospy.loginfo('encoderError() called')
        return
    
    def encoderInputChanged(self, e):
        rospy.loginfo('encoderInputChanged() called')
        rospy.loginfo("left position: %d" % (self.encoder.getPosition(
                self.leftEncoder
                )))
        rospy.loginfo("right position: %d" % (self.encoder.getPosition(
                self.rightEncoder
                )))

        return
    
if __name__ == "__main__":
    print "Starting wheel_encoders"
    rospy.init_node('wheel_encoders', log_level = rospy.DEBUG)
    print "init_node() called"

    rospy.loginfo('Starting wheel_encoders node')

    encoder = PhidgetEncoders()

    while not rospy.is_shutdown():
        rospy.spin()

