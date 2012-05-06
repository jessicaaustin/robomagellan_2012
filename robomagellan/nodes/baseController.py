#!/usr/bin/env python

"""

baseController

- subscribes to the cmd_vel topic to recieve Twist messages
- uses the parameters in the Twist message to translate and
  rotate the rover base

listens to:
 /cmd_vel

publishes to:

"""

import roslib; roslib.load_manifest('robomagellan')
import rospy

from rospy.exceptions import ROSInitException

from geometry_msgs.msg import Twist


