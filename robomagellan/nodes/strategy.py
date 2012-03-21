#!/usr/bin/env python

"""

strategy
- determines where to move the robot next

"""

import roslib; roslib.load_manifest('robomagellan')
import rospy

from waypoint_reader import WaypointFileReader

import os
import sys

class Strategizer():
    def __init__(self, waypoints_file):
        waypoint_file_reader = WaypointFileReader()
        self.waypoints = waypoint_file_reader.read_file(waypoints_file)
        rospy.loginfo("Strategizer loaded with waypoints:\n %s" % self.waypoints)


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

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        rate.sleep()
            
