#!/usr/bin/env python

"""

navigator_capture_cone
  based on the location of a cone published by the cone_tracker node,
  this node will send commands to the robot base on the /cmd_vel topic,
  as it attempts to reach and physically contact the cone.

implements actionlib for:
 /capture_cone

listens to:
 /cone_coord
 /collision
 /odom

publishes to:
 /cmd_vel

"""

import roslib; roslib.load_manifest('robomagellan')
import rospy

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped
from robomagellan.msg import BoolStamped
from nav_msgs.msg import Odometry

from navigator import ConeCaptureNavigator


if __name__ == '__main__':
    rospy.init_node('navigator_capture_cone')
    rospy.sleep(2)  # let rxconsole boot up
    rospy.loginfo("Initializing navigator_capture_cone node")

    # create a navigator for capturing cone waypoints
    cone_navigator = ConeCaptureNavigator('capture_cone')
    rospy.Subscriber('cone_coord', PointStamped, cone_navigator.setup_cone_coord_callback())
    rospy.Subscriber('collision', BoolStamped, cone_navigator.setup_collision_callback())
    rospy.Subscriber('odom', Odometry, cone_navigator.setup_odom_callback())
            
    rospy.spin()
