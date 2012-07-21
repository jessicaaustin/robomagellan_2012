#
# waypoint_reader
# - understands how to read in waypoints from a waypoints file or a world file
# 
# reading from the world file allows us to mock "drift" in our odometry
#  - that is, you can update the location of cone waypoints in the world file
#    to be slightly offset from waypoints in your waypoints file (which is given
#    to the navigation stack)
#

import roslib; roslib.load_manifest('robomagellan')

import rospy
from geometry_msgs.msg import Point

from robomagellan.msg import Waypoint

class WaypointFileReader():
    def __init__(self):
        return
    
    def read_file(self, filename):
        rospy.loginfo('Reading waypoints from file %s' % filename)
        if filename.endswith("waypoints"):
            return self.read_waypoints_file(filename)
        else:
            return self.read_world_file(filename)

    def read_waypoints_file(self, filename):
        rospy.loginfo("File is a waypoints file")
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

    def read_world_file(self, filename):
        rospy.loginfo("File is a world file")
        waypoints = []
        file = open(filename)
        line = file.readline()

        # find the rover line
        while not line.startswith("rover"):
            line = file.readline()

        # rover line gives offset from origin
        args = line.split()
        x_offset = float(args[3])
        y_offset = float(args[4])

        # find the first waypoint/cone line
        while not (line.startswith("waypoint") or line.startswith("cone")):
            line = file.readline()
        
        while len(line) > 0:
            args = line.split()
            x = float(args[3]) - x_offset
            y = float(args[4]) - y_offset
            type = "P"
            if args[0].startswith("cone"):
                type = "C"
            waypoints.append(Waypoint(type, Point(x, y, 0.0)))
            line = file.readline() 
        rospy.loginfo('Waypoints: \n%s' % waypoints)
        rospy.loginfo('len=%s' % len(waypoints))
        return waypoints


