#!/usr/bin/env python

"""

ultrasound_to_laser
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
        return

    def setup_range_callback(self, publisher):
        def range_callback(data):
            # for now we'll only check a single scanner
            if (data.header.frame_id == "ultrasonic_1"):
                self.publish_laser_data(data, publisher)
        return range_callback
                    
    def publish_laser_data(self, range_data, publisher):
        scan = LaserScan()
        scan.header.stamp = rospy.Time.now()
        scan.header.frame_id = range_data.header.frame_id
        scan.angle_min = -1 * math.pi / 180
        scan.angle_max = 1 * math.pi / 180
        scan.angle_increment = 1 * math.pi / 180 
        scan.range_min = 0.03 
        scan.range_max = 3.00
        scan.ranges = [range_data.range]
        publisher.publish(scan)


if __name__ == '__main__':
    rospy.init_node('ultrasound_to_laser')
    rospy.sleep(3)  # let rxconsole boot up
    rospy.loginfo("Initializing ultrasound_to_laser node")

    converter = RangeToLaserConverter()
    publisher = rospy.Publisher('base_scan', LaserScan)
    rospy.Subscriber('bogies', Range, converter.setup_range_callback(publisher))
            
    rospy.spin()
