#!/usr/bin/env python

import serial

class IMU():
    """IMU - read a SparkFun 9DOF and publish the roll, pitch and yaw

    Author: Bill Mania <bill@manialabs.us>

IMU(serialPort)
    serialPort - the serial port where the IMU is connected

getOrientation()
    returns a 4-tuple:

    (roll, pitch, yaw, status)

"""

    def __init__(self, serialPort):
        """
        for reading the roll, pitch and yaw values from a SparkFun
        9DOF
        """

        self.serialPort = serial.Serial(port = serialPort,
            baudrate = 57600,
            timeout = 1
            )
        self.serialPort.setRTS(level = True)
        self.serialPort.setDTR(level = True)
        # self.serialPort.flushInput()

        self.expectedSequenceNumber = 0

        #
        # read whatever is left of the most recent message
        #
        characterRead = self.serialPort.read(1)
        while (characterRead != '\r'):
            characterRead = self.serialPort.read(1)


    def getOrientation(self):
        """read the roll, pitch and yaw values and return them as a tuple
        """

        #
        # read a whole message
        #
        inputMessageBuffer = ''
        characterRead = self.serialPort.read(1)
        while (characterRead != '\r'):
            inputMessageBuffer = inputMessageBuffer + characterRead
            characterRead = self.serialPort.read(1)

        try:
            firstSplit = inputMessageBuffer.split(':')
            sequenceNumber = int(firstSplit[0])
            expectedSequenceNumber = self.expectedSequenceNumber
            self.expectedSequenceNumber = sequenceNumber + 1
            if sequenceNumber != expectedSequenceNumber:
                return (
                    0.0,
                    0.0,
                    0.0,
                    'Sequence. Expected %d, got %d' % (
                        expectedSequenceNumber,
                        sequenceNumber
                        )
                        )

            values = firstSplit[1].split(',')

            roll = float(values[0])
            pitch = float(values[1])
            yaw = float(values[2])

        except:
            return (0.0, 0.0, 0.0, inputMessageBuffer)

        return (roll, pitch, yaw, None)

if __name__== '__main__':

    imu = IMU('/dev/ttyUSB0')

    while True:
        results = imu.getOrientation()

        if (results == None):
            print "No data available"

        print "roll:%f, pitch:%f, yaw:%f\n" % (results[0], results[1], results[2])
