#!/usr/bin/env python

"""

strategy
- determines where to move the robot next

listens to:
 /odom
 /cone_coord

publishes to:
 /move_base_simple/goal

"""

import roslib; roslib.load_manifest('robomagellan')
import rospy

from waypoint_reader import WaypointFileReader
from geometry_msgs.msg import PointStamped

from nav_msgs.msg import Odometry

import os
import sys

class Strategizer():
    def __init__(self, waypoints_file):
        self.current_pos = None
        self.cone_coord = None
        waypoint_file_reader = WaypointFileReader()
        self.waypoints = waypoint_file_reader.read_file(waypoints_file)
        rospy.loginfo("Strategizer loaded with waypoints:\n %s" % self.waypoints)

    def setup_odom_callback(self):
        def odom_callback(data):
            self.current_pos = data.pose.pose.position
        return odom_callback

    def setup_cone_coord_callback(self):
        def cone_coord_callback(data):
            self.cone_coord = data.point
        return cone_coord_callback

if __name__ == '__main__':
    rospy.init_node('strategy')
    rospy.sleep(3)  # let rxconsole boot up
    rospy.loginfo("Initializing strategy node")

    if len(sys.argv) < 2 or not os.path.exists(sys.argv[1]) or not os.path.isfile(sys.argv[1]):
        rospy.logerr("Must supply waypoints file!")
        sys.exit(1)
    else:
        waypoints_file = sys.argv[1]
        strategizer = Strategizer(waypoints_file)
        rospy.Subscriber('odom', Odometry, strategizer.setup_odom_callback())
        rospy.Subscriber('cone_coord', PointStamped, strategizer.setup_cone_coord_callback())

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        rate.sleep()
            
