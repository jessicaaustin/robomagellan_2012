#!/usr/bin/env python

# Read GPS data from Garmin device via gpsbabel, and extract out
#   lat, lng, vel, and heading
#
# Uses pyproj to convert geographic coords (lat,lng) to local map coords (x,y)
#   see http://code.google.com/p/pyproj/
#
#
# TODO publish in radians instead of degrees. should go from -pi to pi, with 
#      positive orientation being counterclockwise (z-up)
#

import roslib; roslib.load_manifest('robomagellan')

import sys
import math
from subprocess import Popen, PIPE, STDOUT
from pyproj import Proj
import os

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import String

import settings


class GpsLocation():
    def __init__(self):
        rospy.loginfo('projection zone=%d' % settings.UTM_ZONE)
        self.projection = Proj({'proj':'utm', 'zone':settings.UTM_ZONE, 'ellps':'WGS84'})
        self.init_x = None
        self.init_y = None
        self.publisher = rospy.Publisher('odom', Odometry)

    def knot_to_vel(self, num):
        return settings.KNOT_TO_M_S * num

    def lat_lng_to_course_frame(self, lat,lng):
        return self.projection(lat,lng)

    def publish_location(self, nmea_str):
        nmea_arr = nmea_str.split(",")
        lat = (float(nmea_arr[3]) / 100) + settings.LAT_OFFSET
        lng = (-1 * float(nmea_arr[5]) / 100) + settings.LNG_OFFSET
        vel = float(nmea_arr[7])
        heading = float(nmea_arr[8])

        rospy.logdebug('lat,lng=(%f,%f)' % (lat,lng))
        rospy.logdebug('vel=%f,heading=%f' % (vel,heading))

        if self.init_x is None:
            self.init_x,self.init_y = self.lat_lng_to_course_frame(lat,lng)
            rospy.loginfo('init_x=%f, init_y=%f' % (self.init_x, self.init_y))

        x,y = self.lat_lng_to_course_frame(lat,lng)

        odom = Odometry()

        updated_x = x - self.init_x
        updated_y = y - self.init_y
        rospy.logdebug('updated_x=%f, updated_y=%f' % (updated_x, updated_y))
        odom.pose.pose.position.x = updated_x
        odom.pose.pose.position.y = updated_y
        odom.twist.twist.linear.x = self.knot_to_vel(vel)
        # TODO convert to radians. right now it's in degrees, but navigation node expects radians
        odom.pose.pose.orientation.z = heading

        self.publisher.publish(odom)


if __name__ == '__main__':
    CMD = 'gpsbabel -T -i garmin -f usb: -o nmea -F -'
    if len(sys.argv) > 1:
        CMD = sys.argv[1]
    
    rospy.loginfo('Running gps with command: %s' % CMD)

    rospy.init_node('gps_location')
    gps_location = GpsLocation()
    pub_raw = rospy.Publisher('gps_raw_data', String)

    p = Popen(CMD, stdout = PIPE, stderr = STDOUT, shell = True)
    while not rospy.is_shutdown():
        line = p.stdout.readline()
        if not line:
            rospy.logerr("Could not read from GPS!")
            break
        pub_raw.publish(line)
        if "GPRMC" in line:
            gps_location.publish_location(line)

