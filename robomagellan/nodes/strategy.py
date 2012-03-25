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
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
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

    def publish_waypoint_markers(self):
        pub_waypoint_marker = rospy.Publisher('waypoint_markers', MarkerArray)
        marker_array = MarkerArray()
        for i in range(len(self.waypoints)):
            waypoint = self.waypoints[i]
            waypoint_marker = Marker()
            waypoint_marker.header.frame_id = "/odom"
            waypoint_marker.header.stamp = rospy.Time.now()
            waypoint_marker.pose.position = waypoint.coordinate
            if (waypoint.type == 'P'):
                waypoint_marker.type = 2  # Sphere
                waypoint_marker.text = 'waypoint_%s_point' % i
                waypoint_marker.color.r = 176.0
                waypoint_marker.color.g = 224.0
                waypoint_marker.color.b = 230.0
                waypoint_marker.color.a = 1.0
                waypoint_marker.scale.x = 5.0
                waypoint_marker.scale.y = 5.0
                waypoint_marker.scale.z = 5.0
            else:
                waypoint_marker.type = 3  # Cylinder
                waypoint_marker.text = 'waypoint_%s_cone' % i
                waypoint_marker.color.r = 255.0
                waypoint_marker.color.g = 69.0
                waypoint_marker.color.b = 0.0
                waypoint_marker.color.a = 1.0
                waypoint_marker.scale.x = 1.3
                waypoint_marker.scale.y = 1.3
                waypoint_marker.scale.z = 1.5
            marker_array.markers.append(waypoint_marker)
        pub_waypoint_marker.publish(marker_array)
                       
        

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
        strategizer.publish_waypoint_markers()
        rate.sleep()
            
