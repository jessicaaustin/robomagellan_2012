#!/usr/bin/env python

import roslib; roslib.load_manifest('robomagellan')

import rospy
from sensor_msgs.msg import Range
import sys
import serial

class Rangefinder():
    def __init__(self):

        rospy.loginfo('Initializing Rangefinder')

        self.serialPort = '/dev/ttyACM0'

        while True:
            try:
                self.rangefinderDevice = serial.Serial(
                    port = self.serialPort,
                    baudrate = 9600,
                    timeout = 1
                    )
                self.rangefinderDevice.setRTS(level = True)
                self.rangefinderDevice.setDTR(level = True)
                self.rangefinderDevice.flushInput()
                break
    
            except serial.SerialException as e:
                rospy.logwarn('Unable to open %s' % (self.serialPort))
                rospy.sleep(15.0)

            except serial.SerialTimeoutException as e:
                rospy.logwarn('Timeout waiting to open %' % (self.serialPort))
                rospy.sleep(15.0)

        self.rangeMessage = Range()
        self.rangeMessage.radiation_type = Range.ULTRASOUND
        self.rangeMessage.field_of_view = 0.018
        self.rangeMessage.min_range = 0.03
        self.rangeMessage.max_range = 3.0
        self.rangeMessage.range = 0.0

        self.publisher = rospy.Publisher('bogies', Range)

        rospy.loginfo('Initialization complete')

    def readNextMessage(self):
        response = ''
        try:
            characterRead = self.rangefinderDevice.read(1)
            while characterRead != '\n':
                if characterRead not in [ '\r', '\n' ]:
                    response = response + characterRead
                characterRead = self.rangefinderDevice.read(1)
    
#            rospy.logdebug('Read <%s> from rangefinder' % (response))
        except:
            pass

        return response.split(',')

    def processRangefinderMessages(self):
        while not rospy.is_shutdown():
            try:
	            rangefinderUnit, distance = self.readNextMessage()
	            self.rangeMessage.header.stamp = rospy.Time.now()
	            self.rangeMessage.header.frame_id = 'ultrasonic_%s' % (rangefinderUnit)
	            self.rangeMessage.range = float(distance)
	            self.publisher.publish(self.rangeMessage)
            except:
                pass

        return

if __name__ == '__main__':

    rospy.init_node('rangefinders', log_level = rospy.DEBUG)

    rangefinder = Rangefinder()
    
    rospy.loginfo('Entering processing loop')
    rangefinder.processRangefinderMessages()

    rospy.logwarn('Stopping')

    sys.exit(0)
