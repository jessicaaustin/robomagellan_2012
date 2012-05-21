#!/usr/bin/env python

# 
# robomagellan_localization.py
#
# This node provides the transform between the /world frame and the 
# /odom frame. See http://www.ros.org/reps/rep-0105.html for more information
# about how these frames are defined.
#
# Localization during the Robomagellan competition is difficult in that
# a) there is no known map beforehand, so something like amcl (Monte Carlo
# localization using a map + particle filter) is not a possibility, and 
# b) GPS is expected to have high covariances and generally be unreliable.
# Thus, the odom frame will be allowed to drift without bound until the robot
# comes in contact with a cone, at which point the location is known again.
#
# The localization provided by this node requires the presence of a cone
# with a known location. Every time a cone is encountered, the transform
# between /world->/odom is updated with the current offset, and that offset is
# broadcast until the next cone is encountered.
#
# For example, at the start of the competition, the /odom frame is linked to
# the /world frame. Then, if we hit a cone known to be at (x=10,y=15) and our
# current /odom is publishing a location of (x=11,y=14.5), the this node
# would start broadcasting a transform of (1,-0.5) for the /world->/odom mapping,
# and continue to do so until the next cone is enountered.
#

import roslib; roslib.load_manifest('robomagellan')
import rospy

import tf

from geometry_msgs.msg import Point

class RobomagellanLocalization():
    def __init__(self):
        # we start with zero offset in our position
        self.current_position_offset = Point()
        # we don't care about drifts in orientation,
        # so we'll just publish a constant for that
        self.unit_quaternion = (0.0, 0.0, 0.0, 1.0)
        rospy.loginfo("RobomagellanLocalization initialized")

    def publish_localization(self):
        br = tf.TransformBroadcaster()
        p = self.current_position_offset
        br.sendTransform((p.x, p.y, p.z),
                         self.unit_quaternion,
                         rospy.Time.now(),
                         "odom",
                         "map")

if __name__ == '__main__':
    rospy.init_node('robomagellan_localization')
    rospy.sleep(3)  # let rxconsole boot up
    rospy.loginfo("Initializing localization node")

    localization = RobomagellanLocalization()

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        rate.sleep()
        localization.publish_localization()
            
