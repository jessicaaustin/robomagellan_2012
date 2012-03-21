#
# waypoint_reader
# - understands how to read in waypoints from a file
#

import roslib; roslib.load_manifest('robomagellan')

import rospy
from geometry_msgs.msg import Point

from robomagellan.msg import Waypoint

class WaypointFileReader():
    def __init__(self):
        return
    
    # TODO convert from GPS to course frame when reading from file
    def read_file(self, filename):
        rospy.loginfo('Reading waypoints from file %s' % filename)
        waypoints = []
        file = open(filename)
        line = file.readline()
        while len(line) > 0:
            args = line.split()
            waypoints.append(Waypoint(args[0], Point(float(args[1]), float(args[2]), 0.0)))
            line = file.readline() 
        rospy.loginfo('Waypoints: \n%s' % waypoints)
        return waypoints

