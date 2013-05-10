#!/usr/bin/env python

"""
PhidgetsEncoders() - Read the encoder values from a
	Phidgets 1047, filter the data a bit and then
	publish the Odometry message.

    Looking at the top of the encoder board, with the
    digital input terminals on the right and the
    encoder inputs on the top, encoder port number 0
    is at the top right and port 3 is at the top left.

"""

import roslib; roslib.load_manifest('robomagellan')
import rospy
from std_msgs.msg import Int64, Int16

from Phidgets.Devices.Encoder import Encoder
from Phidgets.PhidgetException import PhidgetException

class PhidgetEncoders:

    def __init__(self):

        rospy.loginfo("Initializing PhidgetEncoders")

        self.leftFrontEncoder = 1
        self.rightFrontEncoder = 0
        self.leftRearEncoder = 3
        self.rightRearEncoder = 2
        self.leftSignAdjust = 1 # forward is positive
        self.rightSignAdjust = -1 # forward is negative

        self.leftEncoder = Int64(0)
        self.rightEncoder = Int64(0)

        # publish the Odometry message to describe the current position
        # and orientation of the origin of the base_link frame.
        #
        self.encoder = Encoder()

        self.leftEncoderPublisher = rospy.Publisher('lwheel', Int16)
        self.rightEncoderPublisher = rospy.Publisher('rwheel', Int16)

        self.encoder.setOnAttachHandler(self.encoderAttached)
        self.encoder.setOnDetachHandler(self.encoderDetached)
        self.encoder.setOnErrorhandler(self.encoderError)
        self.encoder.setOnInputChangeHandler(self.encoderInputChanged)
        self.encoder.setOnPositionChangeHandler(self.encoderPositionChange)

        try:
            rospy.logdebug('openPhidget()')
            self.encoder.openPhidget()

        except PhidgetException, e:
            rospy.logerror("openPhidget() failed")
            rospy.logerror("code: %d" % e.code)
            rospy.logerror("message", e.message)

            raise

        try:
            rospy.logdebug('waitForAttach()')
            self.encoder.waitForAttach(10000)

        except PhidgetException, e:
            rospy.logerror("waitForAttach() failed")
            rospy.logerror("code: %d" % e.code)
            rospy.logerror("message", e.message)
    
            raise

        return

    def encoderPositionChange(self, e):
        """Called each time the encoder reports a change to its position."""

        return

    def updatePulseValues(self):

        leftFrontPulses = (self.leftSignAdjust * self.encoder.getPosition(self.leftFrontEncoder))
        leftRearPulses = (self.leftSignAdjust * self.encoder.getPosition(self.leftRearEncoder))
        rightFrontPulses = (self.rightSignAdjust * self.encoder.getPosition(self.rightFrontEncoder))
        rightRearPulses = (self.rightSignAdjust * self.encoder.getPosition(self.rightRearEncoder))

        if leftFrontPulses < leftRearPulses:
            self.leftEncoder.data = leftFrontPulses
        else:
            self.leftEncoder.data = leftRearPulses

        if rightFrontPulses < rightRearPulses:
            self.rightEncoder.data = rightFrontPulses
        else:
            self.rightEncoder.data = rightRearPulses

        return

    def encoderAttached(self, e):
        rospy.loginfo('encoderAttached() called')

        self.encoder.setPosition(
                self.leftFrontEncoder,
                0
                )
        self.encoder.setPosition(
                self.leftRearEncoder,
                0
                )
        self.encoder.setPosition(
                self.rightFrontEncoder,
                0
                )
        self.encoder.setPosition(
                self.rightRearEncoder,
                0
                )
        self.encoder.setEnabled(
                self.leftFrontEncoder,
                True
                )
        self.encoder.setEnabled(
                self.leftRearEncoder,
                True
                )
        self.encoder.setEnabled(
                self.rightFrontEncoder,
                True
                )
        self.encoder.setEnabled(
                self.rightRearEncoder,
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
        return
    
if __name__ == "__main__":
    rospy.init_node(
        name = 'wheel_encoders',
        log_level = rospy.DEBUG
        )
    rospy.sleep(3.0)
    rospy.loginfo("Initializing wheel_encoders.py")

    encoder = PhidgetEncoders()

    consistentFrequency = rospy.Rate(5)
    while not rospy.is_shutdown():
        encoder.updatePulseValues()
#        encoder.leftEncoderPublisher.publish(encoder.leftEncoder)
#        encoder.rightEncoderPublisher.publish(encoder.rightEncoder)
        rospy.logdebug('leftEncoder: %d' % (encoder.leftEncoder.data))
        rospy.logdebug('rightEncoder: %d' % (encoder.rightEncoder.data))

        consistentFrequency.sleep()

