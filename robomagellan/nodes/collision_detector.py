#!/usr/bin/env python

"""

collision_detector
- monitors the robot's distance sensors, and sends out a collision message
  whenever the distance is small enough

listens to:
 /base_scan

publishes to:
 /collision
 /obstacle

"""

import roslib; roslib.load_manifest('robomagellan')
import rospy

from robomagellan.msg import BoolStamped
from sensor_msgs.msg import LaserScan

import settings

class CollisionDetector():
    def __init__(self):
        rospy.loginfo("CollisionDetector loaded")
        
    def setup_collision_info_callback(self):
        publisher = rospy.Publisher('collision', BoolStamped)
        def base_scan_callback(data):
            self.publish_info(data, settings.CONE_CAPTURE_COLLISION_DISTANCE, publisher)
        return base_scan_callback

    def setup_obstacle_info_callback(self):
        publisher = rospy.Publisher('obstacle', BoolStamped)
        def base_scan_callback(data):
            self.publish_info(data, settings.OBSTACLE_DISTANCE, publisher)
        return base_scan_callback

    def publish_info(self, scan_data, threshold_distance, publisher):
        detected = False
        for scan in scan_data.ranges:
            if (scan < threshold_distance): 
                detected = True
                break
        info = BoolStamped()
        info.header.frame_id = "base_link"
        info.header.stamp = rospy.Time.now()
        info.value = detected
        publisher.publish(info)

if __name__ == '__main__':
    rospy.init_node('collision_detector')
    rospy.sleep(3)  # let rxconsole boot up
    rospy.loginfo("Initializing collision_detector node")

    collision_detector = CollisionDetector()
    rospy.Subscriber('base_scan', LaserScan, collision_detector.setup_collision_info_callback())
    rospy.Subscriber('base_scan', LaserScan, collision_detector.setup_obstacle_info_callback())

    rospy.spin()
            
