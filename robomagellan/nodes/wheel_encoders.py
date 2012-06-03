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

        self.leftEncoder = 0
        self.rightEncoder = 1

        self.previousXPosition = 0
        self.previousYPosition = 0

        self.encoder = Encoder()

        self.encoder.setOnAttachHandler(self.encoderAttached)
        self.encoder.setOnDetachHandler(self.encoderDetached)
        self.encoder.setOnErrorhandler(self.encoderError)
        self.encoder.setOnInputChangeHandler(self.encoderInputChanged)
        self.encoder.setOnPositionChangeHandler(self.encoderPositionChange)

        try:
            rospy.logdebug('openPhidget()')
            print "openPhidget()"
            self.encoder.openPhidget()

        except PhidgetException, e:
            rospy.logerror("openPhidget() failed")
            rospy.logerror("code: %d" % e.code)
            rospy.logerror("message", e.message)

            raise

        try:
            rospy.logdebug('waitForAttach()')
            print "waitForAttach()"
            self.encoder.waitForAttach(10000)

        except PhidgetException, e:
            rospy.logerror("waitForAttach() failed")
            rospy.logerror("code: %d" % e.code)
            rospy.logerror("message", e.message)
    
            raise

        rospy.logdebug('Encoder device: %s' % (self.encoder.getDeviceName()))
        rospy.logdebug('        serial: %d' % (self.encoder.getSerialNum()))
        rospy.logdebug('        version: %d' % (self.encoder.getDeviceVersion()))
        rospy.logdebug('        count: %d' % (self.encoder.getEncoderCount()))
        rospy.logdebug('        input count: %d' % (self.encoder.getInputCount()))

        print 'Encoder device: %s' % (self.encoder.getDeviceName())
        print '        serial: %d' % (self.encoder.getSerialNum())
        print '        version: %d' % (self.encoder.getDeviceVersion())
        print '        count: %d' % (self.encoder.getEncoderCount())
        print '        input count: %d' % (self.encoder.getInputCount())
        self.encoderPublisher = rospy.Publisher('wheel_odom', Odometry)

        print "Done initializing"

        return

    def encoderPositionChange(self, e):
        """Called each time the encoder reports a change to its position.
           This method will convert the encoder values to an updated
           position in the base_link frame and then publish an Odometry
           message to the odom topic.
        """

        source = e.device
        rospy.logdebug("Serial %i: Encoder %i Change: %i Time: %i Position: %i" % (
            source.getSerialNum(),
            e.index,
            e.positionChange,
            e.time,
            encoder.getPosition(e.index)
            )
            )
        print "Serial %i: Encoder %i Change: %i Time: %i Position: %i" % (
            source.getSerialNum(),
            e.index,
            e.positionChange,
            e.time,
            encoder.getPosition(e.index)
            )

        return



        #
        # we need from the encoder:
        #
        # which wheel encoder changed
        # what is the new value for that encoder
        # at what time was the encoder sampled
        #
        # we must:
        #
        # initialize the previous encoder value and timestamp for each
        #  encoder
        #
        # calculate the difference between the current and previous
        #  encoder and time values
        #

        if count_delta >= maximal_count / 2:
          return count_delta - maximal_count + 1
        elif count_delta <= -maximal_count / 2:
          return count_delta + maximal_count + 1
        return count_delta

        # The distance and angle calculation sent by the robot seems to
        # be really bad. Re-calculate the values using the raw enconder
        # counts.
        if self._last_encoder_counts:
          count_delta_left = self._normalize_encoder_count(
              self.encoder_counts_left - self._last_encoder_counts[0], 0xffff)
          count_delta_right = self._normalize_encoder_count(
              self.encoder_counts_right - self._last_encoder_counts[1], 0xffff)
          distance_left = count_delta_left * self.ROOMBA_PULSES_TO_M
          distance_right = count_delta_right * self.ROOMBA_PULSES_TO_M
          self.distance = (distance_left + distance_right) / 2.0
          self.angle = (distance_right - distance_left) / robot_types.ROBOT_TYPES['roomba'].wheel_separation
        else:
          self.distance = 0
          self.angle = 0
        self._last_encoder_counts = (self.encoder_counts_left, self.encoder_counts_right)


        # based on otl_roomba by OTL <t.ogura@gmail.com>

        current_time = sensor_state.header.stamp
        dt = (current_time - last_time).to_sec()

        x = cos(angle) * d
        y = -sin(angle) * d

        last_angle = self._pos2d.theta
        self._pos2d.x += cos(last_angle)*x - sin(last_angle)*y
        self._pos2d.y += sin(last_angle)*x + cos(last_angle)*y
        self._pos2d.theta += angle

        # Turtlebot quaternion from yaw. simplified version of tf.transformations.quaternion_about_axis
        # odom_quat = (0., 0., sin(self._pos2d.theta/2.), cos(self._pos2d.theta/2.))


        # update the odometry state
    
        encoder_odometry = Odometry()

        encoder_odometry.header.stamp = current_time
        encoder_odometry.pose.pose   = Pose(Point(self._pos2d.x, self._pos2d.y, 0.), Quaternion(*odom_quat))
        encoder_odometry.twist.twist = Twist(Vector3(d/dt, 0, 0), Vector3(0, 0, angle/dt))
        if sensor_state.requested_right_velocity == 0 and \
               sensor_state.requested_left_velocity == 0 and \
               sensor_state.distance == 0:
            encoder_odometry.pose.covariance = ODOM_POSE_COVARIANCE2
            encoder_odometry.twist.covariance = ODOM_TWIST_COVARIANCE2
        else:
            encoder_odometry.pose.covariance = ODOM_POSE_COVARIANCE
            encoder_odometry.twist.covariance = ODOM_TWIST_COVARIANCE

        return

    def encoderAttached(self, e):
        return
    
    def encoderDetached(self, e):
        return
    
    def encoderError(self, e):
        return
    
    def mcCurrentChanged(self, e):
        return
    
    def encoderInputChanged(self, e):
        return
    
    def mcVelocityChanged(self, e):
        return

if __name__ == "__main__":
    print "Starting wheel_encoders"
    rospy.init_node('wheel_encoders', log_level = rospy.DEBUG)
    print "init_node() called"

    rospy.logdebug('Starting wheel_encoders node')

    encoder = PhidgetEncoders()

    print "spin()"
    while not rospy.is_shutdown():
        rospy.spin()

