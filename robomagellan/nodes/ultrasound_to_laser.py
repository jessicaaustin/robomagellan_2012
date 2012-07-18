#!/usr/bin/env python

"""

ultrasound_to_laser
 (AKA "the poor man's laser scanner")
 the move_base node currently only supports LaserScan and PointCloud message types
 this node converts our array of ultrasonic sensors (Range messages) to a LaserScan message

listens to:
 /bogies

publishes to:
 /base_scan

"""

import roslib; roslib.load_manifest('robomagellan')
import rospy

from sensor_msgs.msg import Range
from sensor_msgs.msg import LaserScan

import math

class RangeToLaserConverter():
    def __init__(self):
        self.resetRanges()

    def setup_range_callback(self, publisher):
        def range_callback(data):
            self.ranges[data.header.frame_id] = data.range
            # once we have data from the entire "array", we should publish it
            if (self.hasAllRanges()):
                self.publish_laser_data(publisher)
        return range_callback
                    
    def publish_laser_data(self, publisher):
        scan = LaserScan()
        scan.header.stamp = rospy.Time.now()
        scan.header.frame_id = "base_laser_link"
        scan.angle_min = -30 * math.pi / 180
        scan.angle_max = 30 * math.pi / 180
        scan.angle_increment = 1 * math.pi / 180 
        scan.range_min = 0.03 
        scan.range_max = 3.00
        scan.ranges = []
        # read the ranges from left to right
        for i in range(0,15):
            scan.ranges.append(self.ranges["ultrasonic_3"])
        for i in range(15,30):
            scan.ranges.append(self.ranges["ultrasonic_1"])
        for i in range(30,45):
            scan.ranges.append(self.ranges["ultrasonic_2"])
        for i in range(45,60):
            scan.ranges.append(self.ranges["ultrasonic_4"])
        publisher.publish(scan)
        # clear out the scans
        self.resetRanges()

    def resetRanges(self):
        self.ranges = { "ultrasonic_1": None,
                        "ultrasonic_2": None,
                        "ultrasonic_3": None,
                        "ultrasonic_4": None } 

    def hasAllRanges(self):
        return self.ranges["ultrasonic_1"] and \
                self.ranges["ultrasonic_2"] and \
                self.ranges["ultrasonic_3"] and \
                self.ranges["ultrasonic_4"]


if __name__ == '__main__':
    rospy.init_node('ultrasound_to_laser')
    rospy.sleep(3)  # let rxconsole boot up
    rospy.loginfo("Initializing ultrasound_to_laser node")

    converter = RangeToLaserConverter()
    publisher = rospy.Publisher('base_scan', LaserScan)
    rospy.Subscriber('bogies', Range, converter.setup_range_callback(publisher))
            
    rospy.spin()
