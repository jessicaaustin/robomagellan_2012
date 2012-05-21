#!/usr/bin/env python

# Read GPS data from Garmin device via gpsbabel, and extract out
#   lat, lng, vel, and heading
#
# Uses pyproj to convert geographic coords (lat,lng) to local map coords (x,y)
#   see http://code.google.com/p/pyproj/
#

import roslib; roslib.load_manifest('robomagellan')

import sys
import math
from subprocess import Popen, PIPE, STDOUT
from pyproj import Proj

import rospy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String

import settings


class GpsLocation():
    def __init__(self):
        rospy.loginfo("Initializing GpsLocation")
        rospy.loginfo('projection zone=%d' % settings.UTM_ZONE)
        self.projection = Proj({'proj':'utm', 'zone':settings.UTM_ZONE, 'ellps':'WGS84'})
        self.init_x = None
        self.init_y = None
        self.publisher = rospy.Publisher('odom', NavSatFix)

    def knot_to_vel(self, num):
        return settings.KNOT_TO_M_S * num

    # gps module gives us degrees from 0-360
    # here we convert to radians, from -pi to pi, positive counterclockwise (z-up)
    def heading_in_degrees_to_radians(self, heading):
        return -1 * (heading - 180) * math.pi / 180

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
        odom.pose.pose.orientation.z = self.heading_in_degrees_to_radians(heading)

        self.publisher.publish(odom)


if __name__ == '__main__':
    # wait for rxconsole to boot up
    rospy.sleep(3)
    CMD = "gpsbabel -T -i garmin -f usb: -o nmea -F -"
#    rospy.loginfo("CMD: %s" % CMD)
#    if len(sys.argv) > 1:
#        CMD = sys.argv[1]
#        rospy.loginfo("Updating CMD using given argument: %s" % CMD)

    rospy.init_node('gps_location')
    gps_location = GpsLocation()
    pub_raw = rospy.Publisher('gps_raw_data', String)
    rate = rospy.Rate(10.0)
    
    rospy.loginfo("Running gps with command: %s" % CMD)
    p = Popen(CMD, stdout = PIPE, stderr = STDOUT, shell = True)
    while not rospy.is_shutdown():
        line = p.stdout.readline()
        if not line:
            rospy.logerr("Could not read from GPS!")
        else:
            pub_raw.publish(line)
            if "GPRMC" in line:
                gps_location.publish_location(line)
        rate.sleep()

