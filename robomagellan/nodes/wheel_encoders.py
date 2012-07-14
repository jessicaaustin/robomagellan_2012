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
from sensor_msgs.msg import Imu as ImuMessage
from tf import transformations

class PhidgetEncoders:

    def __init__(self):

        rospy.loginfo("Initializing PhidgetEncoders")

        self.leftEncoder = 0 # forward is negative
        self.rightEncoder = 1 # forward is positive
        self.driveWheelRadius = 0.049
        self.wheelSeparation = 0.258
        self.pulsesPerRevolution = 2400 # wheel revolution
        self.wheelsConstant = 2 * pi * self.driveWheelRadius / self.wheelSeparation
        self.pulsesConstant = (pi / self.pulsesPerRevolution) * self.driveWheelRadius

        #
        # these covariance values are complete guesses
        #
        self.defaultCovariance = [1000.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                  0.0, 1000.0, 0.0, 0.0, 0.0, 0.0,
                                  0.0, 0.0, 1000.0, 0.0, 0.0, 0.0,
                                  0.0, 0.0, 0.0, 1000.0, 0.0, 0.0,
                                  0.0, 0.0, 0.0, 0.0, 1000.0, 0.0,
                                  0.0, 0.0, 0.0, 0.0, 0.0, 1000.0]

        #
        # the rover begins with the origins and axes of the odom and
        # base_link frames aligned.
        #
        self.previousX = 0
        self.previousY = 0
        self.previousLeftPosition = 0
        self.previousRightPosition = 0

        #
        # the rover begins oriented along the positive X of both the
        # odom and base_link frames.
        #
        self.previousTheta = 0

        #
        # publish the Odometry message to describe the current position
        # and orientation of the origin of the base_link frame.
        #
        self.encoder = Encoder()
        self.odometryMessage = Odometry()
        self.odometryMessage.header.frame_id = 'base_link'
        self.odometryMessage.child_frame_id = 'base_link'

        #
        # the rover is incapable of translating along the Z axis,
        # so it will always be zero
        #
        self.odometryMessage.pose.pose.position.z = 0

        self.odometryMessage.pose.covariance = self.defaultCovariance
        self.odometryMessage.twist.covariance = self.defaultCovariance

        #
        # initialize this node's idea of the starting yaw orientation
        # by waiting for a message from the IMU
        #
        imuMessage = ImuMessage()
        imuMessage = rospy.wait_for_message(
            'imu_data',
            ImuMessage,
            timeout = None)
        euler = transformations.euler_from_quaternion([
            imuMessage.orientation.x,
            imuMessage.orientation.y,
            imuMessage.orientation.z,
            imuMessage.orientation.w
            ])
        self.previousTheta = euler[2]
        rospy.loginfo('Synchronized with IMU yaw')

        #
        # robot_pose_ekf subscribes (via remapping) to wheel_odom,
        # to get 2D position and orientation data. the z, roll and pitch
        # are ignored by robot_pose_ekf.
        #
        self.encoderPublisher = rospy.Publisher('wheel_odom', Odometry)

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

    def updateOdometry(self):
        """This method will convert the encoder values to an updated
           position in the base_link frame and then publish an Odometry
           message to the wheel_odom topic.

           The math for this method came from:
           http://en.wikipedia.org/wiki/Dead_reckoning#Differential_steer_drive_dead_reckoning

        """

        currentTime = rospy.Time.now()
        deltaT = currentTime.to_sec() - self.odometryMessage.header.stamp.to_sec()
        self.odometryMessage.header.stamp = currentTime

        #
        # calculate the delta between the current encoder value for each wheel and
        # the value from the previous reading.
        #
        leftPulses = (-1 * self.encoder.getPosition(self.leftEncoder)) - self.previousLeftPosition
        rightPulses = self.encoder.getPosition(self.rightEncoder) - self.previousRightPosition
        self.previousLeftPosition += leftPulses
        self.previousRightPosition += rightPulses

        #
        # subtract the leftPulses (current delta) from the rightPulses, in order
        # to make a left turn have a positive deltaTheta.
        #
        deltaTheta = self.wheelsConstant * (rightPulses - leftPulses) / self.pulsesPerRevolution

        #
        # this calculation of theta doesn't handle the overflow
        #
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
        # linear velocity in meters per second, angular velocity
        # in radians per second
        #
        self.odometryMessage.twist.twist.linear.x = deltaX / deltaT
        self.odometryMessage.twist.twist.linear.y = deltaY / deltaT
        self.odometryMessage.twist.twist.linear.z = 0.0
        self.odometryMessage.twist.twist.angular.x = 0.0
        self.odometryMessage.twist.twist.angular.y = 0.0
        self.odometryMessage.twist.twist.angular.z = deltaTheta / deltaT

        #
        # update the records
        #
        self.previousX = self.odometryMessage.pose.pose.position.x
        self.previousY = self.odometryMessage.pose.pose.position.y
        self.previousTheta = theta

        return

    def encoderAttached(self, e):
        rospy.loginfo('encoderAttached() called')

        self.encoder.setPosition(
                self.leftEncoder,
                self.previousLeftPosition,
                )
        self.encoder.setPosition(
                self.rightEncoder,
                self.previousRightPosition,
                )
        self.encoder.setEnabled(
                self.leftEncoder,
                True
                )
        self.encoder.setEnabled(
                self.rightEncoder,
                True
                )

        rospy.loginfo('Encoder device: %s' % (self.encoder.getDeviceName()))
        rospy.loginfo('        serial: %d' % (self.encoder.getSerialNum()))
        rospy.loginfo('        version: %d' % (self.encoder.getDeviceVersion()))
        rospy.loginfo('        count: %d' % (self.encoder.getEncoderCount()))
        rospy.loginfo('        input count: %d' % (self.encoder.getInputCount()))


        #
        # initialize with the first Odometry message
        #
        self.odometryMessage.header.stamp = rospy.Time.now()
        self.encoderPublisher.publish(self.odometryMessage)
        rospy.loginfo('wheel_encoders initialized')

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
    rospy.init_node('wheel_encoders', log_level = rospy.DEBUG)
    rospy.sleep(3.0)
    rospy.loginfo("Initializing wheel_encoders.py")

    encoder = PhidgetEncoders()

    #
    # robot_pose_ekf uses a default update frequency of 30 Hz
    #
    consistentFrequency = rospy.Rate(30)
    rospy.loginfo('Entering updateOdometry() loop')
    while not rospy.is_shutdown():
        encoder.updateOdometry()
        encoder.encoderPublisher.publish(encoder.odometryMessage)
        consistentFrequency.sleep()

