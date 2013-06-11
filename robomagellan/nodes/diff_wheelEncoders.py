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
from std_msgs.msg import Int16

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
        self.rollover = 32768

        self.leftEncoder = Int16(0)
        self.rightEncoder = Int16(0)

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
        rightFrontPulses = (self.rightSignAdjust * self.encoder.getPosition(self.rightFrontEncoder))
        leftRearPulses = (self.leftSignAdjust * self.encoder.getPosition(self.leftRearEncoder))
        rightRearPulses = (self.rightSignAdjust * self.encoder.getPosition(self.rightRearEncoder))

        leftPulses = (leftFrontPulses + leftRearPulses) / 2
        rightPulses = (rightFrontPulses + rightRearPulses) / 2

        rospy.logdebug('leftEncoder: %d, front: %d, rear: %d' % (
            leftPulses,
            leftFrontPulses,
            leftRearPulses
            ))
        rospy.logdebug('rightEncoder: %d, front: %d, rear: %d' % (
            rightPulses,
            rightFrontPulses,
            rightRearPulses
            ))

        #
        # actively manage the encoder value rollover. The encoder value,
        # when rolling forward, will start at 0 and move toward self.rollover.
        # When it exceeds self.rollover, it is reset to -self.rollover plus
        # the amount by which it exceeded self.rollover.
        #
        if leftPulses > self.rollover:
            rospy.logdebug('left rolled over forward')
            leftPulses = -(self.rollover) + leftPulses - self.rollover
            self.encoder.setPosition(
                self.leftFrontEncoder,
                leftPulses
                )
            self.encoder.setPosition(
                self.leftRearEncoder,
                leftPulses
                )
        elif leftPulses < -(self.rollover):
            rospy.logdebug('left rolled over backward')
            leftPulses = self.rollover + leftPulses + self.rollover
            self.encoder.setPosition(
                self.leftFrontEncoder,
                leftPulses
                )
            self.encoder.setPosition(
                self.leftRearEncoder,
                leftPulses
                )
        if rightPulses > self.rollover:
            rospy.logdebug('right rolled over forward')
            rightPulses = -(self.rollover) + rightPulses - self.rollover
            self.encoder.setPosition(
                self.rightFrontEncoder,
                rightPulses
                )
            self.encoder.setPosition(
                self.rightRearEncoder,
                rightPulses
                )
        elif rightPulses < -(self.rollover):
            rospy.logdebug('right rolled over backward')
            rightPulses = self.rollover + rightPulses + self.rollover
            self.encoder.setPosition(
                self.rightFrontEncoder,
                rightPulses
                )
            self.encoder.setPosition(
                self.rightRearEncoder,
                rightPulses
                )

        self.leftEncoder.data = leftPulses
        self.rightEncoder.data = rightPulses

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

        encoder.leftEncoderPublisher.publish(encoder.leftEncoder)
        encoder.rightEncoderPublisher.publish(encoder.rightEncoder)

        consistentFrequency.sleep()

