#!/usr/bin/env python

"""
calibrateEncoders.py

run one motor for a certain number of encoder pulses, in order
to verify/calibrate the encoder

"""


import sys
from Phidgets.Devices.Encoder import Encoder
from Phidgets.Devices.MotorControl import MotorControl
from Phidgets.PhidgetException import PhidgetErrorCodes, PhidgetException

def encoderAttached( e):
    return

def encoderDetached( e):
    return

def encoderError( e):
    return

def encoderInputChanged( e):
    return

def encoderPositionChange( e):
    return

def mcAttached( e):
    return

def mcDetached( e):
    return

def mcError( e):
    return

def mcCurrentChanged( e):
    return

def mcInputChanged( e):
    return

def mcVelocityChanged( e):
    return

encoder = Encoder()
encoder.setOnAttachHandler(encoderAttached)
encoder.setOnDetachHandler(encoderDetached)
encoder.setOnErrorhandler(encoderError)
encoder.setOnInputChangeHandler(encoderInputChanged)
encoder.setOnPositionChangeHandler(encoderPositionChange)

try:
    encoder.openPhidget()

except PhidgetException, e:
    raise

try:
    encoder.waitForAttach(10000)

except PhidgetException, e:
    raise

if encoder.isAttached():
    pass
else:
    print ("encoder attach Failed")

motorControl = MotorControl()
motorControl.setOnAttachHandler(mcAttached)
motorControl.setOnDetachHandler(mcDetached)
motorControl.setOnErrorhandler(mcError)
motorControl.setOnCurrentChangeHandler(mcCurrentChanged)
motorControl.setOnInputChangeHandler(mcInputChanged)
motorControl.setOnVelocityChangeHandler(mcVelocityChanged)

try:
    motorControl.openPhidget()

except PhidgetException, e:
    print "openPhidget() failed"
    print "code: %d" % e.code
    print "message", e.message

    raise

try:
    motorControl.waitForAttach(10000)

except PhidgetException, e:
    print "waitForAttach() failed"
    print "code: %d" % e.code
    print "message", e.message
    
    raise

if motorControl.isAttached():
    pass
else:
    print ("motor attach Failed")

encoder.setPosition(int(sys.argv[4]), 0)
encoder.setEnabled(int(sys.argv[4]), True)
motorControl.setVelocity(int(sys.argv[1]), int(sys.argv[2]))

position = encoder.getPosition(int(sys.argv[4]))
while position < int(sys.argv[3]):
	position = encoder.getPosition(int(sys.argv[4])) * int(sys.argv[5])
	print position

motorControl.setVelocity(int(sys.argv[1]), 0)
print encoder.getPosition(int(sys.argv[1]))


sys.exit(0)
