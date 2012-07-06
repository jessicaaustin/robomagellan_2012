"""Phidgets HC Motor Control ROS class

"""

__author__ = 'Bill Mania <bill@manailabs.us>'
__version__ = '1'

import rospy
import time
from ctypes import *
from Phidgets.Devices.MotorControl import MotorControl
from Phidgets.PhidgetException import PhidgetException


class PhidgetMotorController:

    def __init__(self):
        self.leftWheels = 0
        self.rightWheels = 1
        self.defaultMotorSpeed = 100.0
        self.motorMaxSpeed = 100
        self.motorMinSpeed = 20

        self.motorControl = MotorControl()

        self.motorControl.setOnAttachHandler(self.mcAttached)
        self.motorControl.setOnDetachHandler(self.mcDetached)
        self.motorControl.setOnErrorhandler(self.mcError)
        self.motorControl.setOnCurrentChangeHandler(self.mcCurrentChanged)
        self.motorControl.setOnInputChangeHandler(self.mcInputChanged)
        self.motorControl.setOnVelocityChangeHandler(self.mcVelocityChanged)

        try:
            self.motorControl.openPhidget()

        except PhidgetException, e:
            print "openPhidget() failed"
            print "code: %d" % e.code
            print "message", e.message

            raise

        try:
            self.motorControl.waitForAttach(10000)

        except PhidgetException, e:
            print "waitForAttach() failed"
            print "code: %d" % e.code
            print "message", e.message
    
            raise

        if self.motorControl.isAttached():
            print("Device: %s, Serial: %d, Version: %d" % (
                    self.motorControl.getDeviceName(),
                    self.motorControl.getSerialNum(),
                    self.motorControl.getDeviceVersion()
                    ))
        else:
            print ("Failed")
    
        self.minAcceleration = self.motorControl.getAccelerationMin(self.leftWheels)
        self.maxAcceleration = self.motorControl.getAccelerationMax(self.leftWheels)

        rospy.loginfo('PhidgetMotorController initialized')

        return
        
    def setDefaultSpeed(self, defaultSpeed):
        self.defaultMotorSpeed = defaultSpeed

        return

    def getDefaultSpeed(self):
        return self.defaultMotorSpeed

    def move(self, translationX, rotationZ):
        """Move the rover base
    
        rotationZ specifies the amount to first rotate the rover base
        and then translationX is the amount to translate the rover base.
        the rotation argument is interpreted as radians per second and the
        translation argument is interpreted as meters per second.
    
        this method assumes that the publisher of the Twist message
        will not request a speed which is greater than zero but less
        than self.motorMinSpeed, but this assumption has not been validated.
        """
    
        self.rotate(rotationZ)

        self.translate(translationX)
    
        return
    
    def rotate(self, rotationZ):
        leftSpeed = -(self.defaultMotorSpeed) * rotationZ
        rightSpeed = self.defaultMotorSpeed * rotationZ

        if leftSpeed > self.motorMaxSpeed:
            rospy.logwarn("rotation %f results in excess speed %f" % (
                rotationZ,
                leftSpeed))
            leftSpeed = self.motorMaxSpeed
        elif leftSpeed < -(self.motorMaxSpeed):
            rospy.logwarn("rotation %f results in excess speed %f" % (
                rotationZ,
                leftSpeed))
            leftSpeed = -(self.motorMaxSpeed)

        if rightSpeed > self.motorMaxSpeed:
            rightSpeed = self.motorMaxSpeed
        elif rightSpeed < -(self.motorMaxSpeed):
            rightSpeed = -(self.motorMaxSpeed)
    
        self.motorControl.setVelocity(self.leftWheels, leftSpeed);
        self.motorControl.setVelocity(self.rightWheels, rightSpeed);
    
        return
    
    def translate(self, translationX):

        #
        # these magic numbers are derived from the motor speed and
        # wheel radius
        #
        wheelSpeed = (translationX - 0.12) / 0.0054

        if wheelSpeed > self.motorMaxSpeed:
            rospy.logwarn("translation %f results in excess speed %f" % (
                translationX,
                wheelSpeed))
            wheelSpeed = self.motorMaxSpeed
        elif wheelSpeed < -(self.motorMaxSpeed):
            rospy.logwarn("translation %f results in excess speed %f" % (
                translationX,
                wheelSpeed))
            wheelSpeed = -(self.motorMaxSpeed)
    
        self.motorControl.setVelocity(self.leftWheels, wheelSpeed);
        self.motorControl.setVelocity(self.rightWheels, wheelSpeed);
    
        return
    
    def mcAttached(self, e):
        return
    
    def mcDetached(self, e):
        return
    
    def mcError(self, e):
        return
    
    def mcCurrentChanged(self, e):
        return
    
    def mcInputChanged(self, e):
        return
    
    def mcVelocityChanged(self, e):
        return

if __name__ == "__main__":
    motorControl = PhidgetMotorController()

    while True:
        motorControl.move(0.0, 1.0)
        time.sleep(0.1)
