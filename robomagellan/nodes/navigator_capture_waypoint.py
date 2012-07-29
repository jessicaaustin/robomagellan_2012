#!/usr/bin/env python

"""

navigator_capture_waypoint
  based on the current location of the robot as published to the /odom topic,
  this node will send commands to the robot base on the /cmd_vel topic,
  as it attempts to reach the given waypoint.

implements actionlib for:
 /capture_waypoint

listens to:
 /collision
 /odom

publishes to:
 /cmd_vel

"""

import roslib; roslib.load_manifest('robomagellan')
import rospy

from geometry_msgs.msg import Twist
from robomagellan.msg import BoolStamped
from nav_msgs.msg import Odometry

from navigator import WaypointNavigator


if __name__ == '__main__':
    rospy.init_node('navigator_capture_waypoint')
    rospy.sleep(3)  # let rxconsole boot up
    rospy.loginfo("Initializing navigator_capture_waypoint node")

    # create a navigator for capturing intermediate waypoints
    waypoint_navigator = WaypointNavigator('capture_waypoint')
    rospy.Subscriber('collision', BoolStamped, waypoint_navigator.setup_collision_callback())
    rospy.Subscriber('obstacle', BoolStamped, waypoint_navigator.setup_obstacle_callback())
    rospy.Subscriber('odom', Odometry, waypoint_navigator.setup_odom_callback())
            
    rospy.spin()
