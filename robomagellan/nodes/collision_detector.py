#!/usr/bin/env python

"""

collision_detector
- monitors the robot's distance sensors, and sends out a collision message
  whenever the distance is small enough

listens to:
 /base_scan

publishes to:
 /collision

"""

import roslib; roslib.load_manifest('robomagellan')
import rospy

from robomagellan.msg import BoolStamped
from sensor_msgs.msg import LaserScan

import settings

class CollisionDetector():
    def __init__(self):
        rospy.loginfo("CollisionDetector loaded")
        
    def setup_base_scan_callback(self, publisher):
        def base_scan_callback(data):
            self.publish_collision_info(data, publisher)
        return base_scan_callback

    def publish_collision_info(self, scan_data, publisher):
        collision_detected = False
        for scan in scan_data.ranges:
            if (scan < settings.COLLISION_DISTANCE): 
                collision_detected = True
                break
        collision_stamped = BoolStamped()
        collision_stamped.header.frame_id = "base_link"
        collision_stamped.header.stamp = rospy.Time.now()
        collision_stamped.value = collision_detected
        publisher.publish(collision_stamped)


if __name__ == '__main__':
    rospy.init_node('collision_detector')
    rospy.sleep(3)  # let rxconsole boot up
    rospy.loginfo("Initializing collision_detector node")

    collision_detector = CollisionDetector()
    publisher = rospy.Publisher('collision', BoolStamped)
    rospy.Subscriber('base_scan', LaserScan, collision_detector.setup_base_scan_callback(publisher))

    rospy.spin()
            
