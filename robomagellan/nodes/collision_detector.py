#!/usr/bin/env python

"""

collision_detector
- monitors the robot bump sensor, and publishes to /collision topic whenever a bump is detected

"""

import roslib; roslib.load_manifest('robomagellan')
import rospy

from robomagellan.msg import BoolStamped
from nav_msgs.msg import Odometry

import settings
from waypoint_reader import WaypointFileReader

import os
import sys
import math

class CollisionDetector():
    def __init__(self):
        rospy.loginfo("CollisionDetector loaded")

    def has_collided(self):
        # TODO implement
        return False


class CollisionDetectorSim():
    def __init__(self, waypoints_file):
        waypoint_file_reader = WaypointFileReader()
        self.obstacles = waypoint_file_reader.read_file(waypoints_file)
        self.current_pos = None
        rospy.loginfo('CollisionDetectorSim loaded with obstacles:\n %s' % self.obstacles)

    def setup_odom_callback(self):
        def odom_callback(data):
            self.current_pos = data.pose.pose.position
        return odom_callback

    def has_collided(self):
        if self.current_pos != None:
            for obstacle in self.obstacles:
                return (obstacle.type == 'C' and 
                        math.fabs(self.current_pos.x-obstacle.coordinate.x) < settings.COLLISION_THRESHOLD and
                        math.fabs(self.current_pos.y-obstacle.coordinate.y) < settings.COLLISION_THRESHOLD)
        else:
            rospy.loginfo('Waiting for current position')


if __name__ == '__main__':
    rospy.init_node('collision_detector')
    rospy.sleep(3)  # let rxconsole boot up
    rospy.loginfo("Initializing collision_detector node")

    if len(sys.argv) >= 2 and os.path.exists(sys.argv[1]) and os.path.isfile(sys.argv[1]):
        # running in simulation mode
        waypoints_file = sys.argv[1]
        collision_detector = CollisionDetectorSim(waypoints_file)
        rospy.Subscriber('odom', Odometry, collision_detector.setup_odom_callback())
    else:
        # running in normal mode
        collision_detector = CollisionDetector()

    pub_collision = rospy.Publisher('collision', BoolStamped)
    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        if collision_detector.has_collided():
            collision_stamped = BoolStamped()
            collision_stamped.header.frame_id = "robot_base"
            collision_stamped.header.stamp = rospy.Time.now()
            collision_stamped.value = True
            pub_collision.publish(collision_stamped)
        rate.sleep()
            
