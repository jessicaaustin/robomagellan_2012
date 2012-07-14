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

        # first line is offset from origin
        args = line.split()
        x_offset = float(args[0])
        y_offset = float(args[1])
        
        line = file.readline()
        while len(line) > 0:
            args = line.split()
            x = float(args[1]) - x_offset
            y = float(args[2]) - y_offset
            waypoints.append(Waypoint(args[0], Point(x, y, 0.0)))
            line = file.readline() 
        rospy.loginfo('Waypoints: \n%s' % waypoints)
        return waypoints

