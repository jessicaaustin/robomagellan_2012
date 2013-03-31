#!/usr/bin/env python

"""
calibrateEncoders.py

run one motor for a certain number of encoder pulses, in order
to verify/calibrate the encoder

"""


import sys
import time
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

if len(sys.argv) < 5:
    print "usage:", sys.argv[0], "motor velocity pulses encoder sign"
    sys.exit(1)


whichMotor = int(sys.argv[1])
velocity = float(sys.argv[2])
pulsesWanted = int(sys.argv[3])
whichEncoder = int(sys.argv[4])
encoderSign = int(sys.argv[5])

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
    print "encoder attached"
else:
    print "encoder attach Failed"

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
    print "motor control attached"
else:
    print "motor attach Failed"

encoder.setPosition(whichEncoder, 0)
encoder.setEnabled(whichEncoder, True)
averageAcceleration = (motorControl.getAccelerationMax(whichMotor) - motorControl.getAccelerationMin(whichMotor)) * 0.9
print "averageAcceleration", averageAcceleration
motorControl.setAcceleration(whichMotor, averageAcceleration)

position = encoder.getPosition(whichEncoder)
print "Starting position", position
startTime = time.clock()
motorControl.setVelocity(whichMotor, velocity)
try:
    while position < pulsesWanted:
        position = encoder.getPosition(whichEncoder) * encoderSign
        print "Position", position, "Current", motorControl.getCurrent(whichMotor), "\r",
    
except:
    print "Aborted"
    pass
    
endTime = time.clock()
motorControl.setVelocity(whichMotor, 0)

print ""
print "Position", encoder.getPosition(whichEncoder) * encoderSign
print "Pulses per second", position / (endTime - startTime)
print "Current", motorControl.getCurrent(whichMotor)
time.sleep(1.0)
print "Current", motorControl.getCurrent(whichMotor)


sys.exit(0)
