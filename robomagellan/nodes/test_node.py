#!/usr/bin/env python

#
# test out the move_base node by sending a simple goal
#

import roslib; roslib.load_manifest('robomagellan')
import rospy

from geometry_msgs.msg import PoseStamped

if __name__ == '__main__':
    rospy.init_node('test_node')
    rospy.sleep(3)  # let rxconsole boot up
    rospy.loginfo("Initializing test node")

    pub_collision = rospy.Publisher('move_base_simple/goal', PoseStamped)
    rate = rospy.Rate(1.0)

    while not rospy.is_shutdown():
        goal = PoseStamped()
        goal.header.frame_id = "/odom"
        goal.pose.position.x = 1.0
        goal.pose.position.y = 0.0
        goal.pose.position.z = 0.0
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0
        pub_collision.publish(goal)
        rate.sleep()
            
