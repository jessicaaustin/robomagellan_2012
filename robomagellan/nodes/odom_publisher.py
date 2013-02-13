#!/usr/bin/env python

# 
# odom_publisher.py
#
# subscribes to:
#  /robot_pose_ekf/odom
#
# publishes to:
# /odom
#
# this logic was separated out of robomagellan_localization,
# in order to:
# 
# 1. receive the PoseWithCovarianceStamped messages on
#     robot_pose_efk/odom from robot_pose_ekf
# 2. add velocity information and create an Odometry message
# 3. publish the Odometry messages to the odom topic
#

import roslib; roslib.load_manifest('robomagellan')
import rospy

import tf

from geometry_msgs.msg import Point, PoseWithCovarianceStamped, Twist, Vector3
from nav_msgs.msg import Odometry

class RobomagellanLocalization():
    def __init__(self, odom_publisher):
        self.odom_publisher = odom_publisher
        # we start with zero offset in our position
        self.current_position_offset = Point()
        # we don't care about drifts in orientation,
        # so we'll just publish a constant for that
        self.unit_quaternion = (0.0, 0.0, 0.0, 1.0)
        self.last_time = None
        # store the latest odometry
        self.odom = None
        self.transformListener = tf.TransformListener()
        self.br = tf.TransformBroadcaster()
        rospy.loginfo("RobomagellanLocalization initialized")

    def publish_odometry(self, odom_combined):
        """
            use current pose, along with delta from last pose to publish 
            the current Odometry on the /odom topic
        """

        if not self.last_time:
            # set up initial times and pose
            rospy.loginfo("Setting up initial position")
            self.last_time, self.last_x, y, self.last_theta = self.current_pose(odom_combined)
            return

        # publish to the /odom topic
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "/base_link"
        odom.pose = odom_combined.pose

        current_time, x, y, theta = self.current_pose(odom_combined)
        dt = current_time - self.last_time
        dt = dt.to_sec()
        d_x = x - self.last_x
        d_theta = theta - self.last_theta
        odom.twist.twist = Twist(Vector3(d_x/dt, 0, 0), Vector3(0, 0, d_theta/dt))

        self.odom_publisher.publish(odom)

        self.last_time, self.last_x, self.last_theta = current_time, x, theta

    def current_pose(self, odom_combined):
        current_time = odom_combined.header.stamp
        x = odom_combined.pose.pose.position.x
        y = odom_combined.pose.pose.position.y
        theta = self.yaw_from_quaternion(odom_combined.pose.pose.orientation)
        return (current_time, x, y, theta)

    def yaw_from_quaternion(self, q):
        roll, pitch, yaw = tf.transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))
        return yaw

    def setup_odom_combined_callback(self):
        def odom_combined_callback(data):
            self.odom = data
            self.publish_odometry(data)
        return odom_combined_callback

if __name__ == '__main__':
    rospy.init_node('odom_publisher')
    rospy.sleep(3)  # let rxconsole boot up
    rospy.loginfo("Initializing odom_publisher node")

    odom_publisher = rospy.Publisher('/odom', Odometry)
    localization = RobomagellanLocalization(odom_publisher)
    rospy.Subscriber('/robot_pose_ekf/odom', PoseWithCovarianceStamped, localization.setup_odom_combined_callback())

    while not rospy.is_shutdown():
        rospy.spin()
