#!/usr/bin/env python

"""Phidgets dual encoders

"""

import roslib; roslib.load_manifest('robomagellan')
import rospy

#from ctypes import *
from math import pi, cos, sin
from Phidgets.Devices.Encoder import Encoder
from Phidgets.PhidgetException import PhidgetException
from nav_msgs.msg import Odometry
from tf import transformations

class PhidgetEncoders:

    def __init__(self):

        print "Initializing PhidgetEncoders"

        self.leftEncoder = 0 # forward is negative
        self.rightEncoder = 1 # forward is positive
        self.driveWheelRadius = 0.055
        self.wheelSeparation = 0.26
        self.pulsesPerRevolution = 1024
        self.wheelsConstant = 2 * pi * self.diveWheelRadius / self.wheelSeparation
        self.pulsesConstant = (pi / self.pulsesPerRevolution) * self.driveWheelRadius

        self.previousX = 0
        self.previousY = 0
        self.previousTheta = 0

        self.encoder = Encoder()
        self.odometryMessage = Odometry()
        self.odometryMessage.header.frame_id = 'base_footprint'
        self.odometryMessage.pose.pose.position.z = 0

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

        rospy.loginfo('wheel_encoders initialized')

        return

    def encoderPositionChange(self, e):
        """Called each time the encoder reports a change to its position.
           This method will convert the encoder values to an updated
           position in the base_link frame and then publish an Odometry
           message to the wheel_odom topic.

           The math for this method came from:
           http://en.wikipedia.org/wiki/Dead_reckoning#Differential_steer_drive_dead_reckoning

        """
        rospy.logdebug('Encoder %i Change: %i Time: %i' % (
            e.index,
            e.positionChange,
            e.time
            )
            )

        self.odometryMessage.header.stamp = rospy.Time.now()
        #
        # this function is called each time any encoder reports
        # a change in position. the next section gets the current
        # position change of the "other" encoder, so it will always have
        # both encoder positions at generally the same time.
        #
        if e.index == self.leftEncoder:
            leftPulses = -1 * e.positionChange
            rightPulses = self.encoder.getPosition(self.rightEncoder)
        else:
            rightPulses = e.positionChange
            leftPulses = -1 * self.encoder.getPosition(self.leftEncoder)

        deltaTheta = self.wheelsConstant * (leftPulses - rightPulses) / self.pulsesPerRevolution
        theta = self.previousTheta + deltaTheta
        deltaX = cos(theta) * (leftPulses + rightPulses) * self.pulsesConstant
        deltaY = sin(theta) * (leftPulses + rightPulses) * self.pulsesConstant

        #
        # determine the current Pose and Twist
        #
        self.odometryMessage.pose.pose.position.x = self.previousX + deltaX
        self.odometryMessage.pose.pose.position.y = self.previousY + deltaY
        odometryQuaternion = transformations.quaternion_from_euler(0, 0, theta)
        self.odometryMessage.pose.pose.orientation.x = odometryQuaternion[0]
        self.odometryMessage.pose.pose.orientation.y = odometryQuaternion[1]
        self.odometryMessage.pose.pose.orientation.z = odometryQuaternion[2]
        self.odometryMessage.pose.pose.orientation.w = odometryQuaternion[3]

        #
        # publish to wheel_odom
        #
        self.encoderPublisher(self.odometryMessage)

        #
        # update the records
        #
        self.previousX = self.previousX + deltaX
        self.previousY = self.previousY + deltaY
        self.previousTheta = self.previousTheta + deltaTheta

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

