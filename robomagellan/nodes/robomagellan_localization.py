#!/usr/bin/env python

# 
# robomagellan_localization.py
#
# This node provides two things:
# - the transform between the /world frame and the /odom frame. 
#   See http://www.ros.org/reps/rep-0105.html for more information
#   about how these frames are defined.
# - Odometry information on the /odom topic 
# 
# Localization during the Robomagellan competition is difficult in that
# a) there is no known map beforehand, so something like amcl (Monte Carlo
# localization using a map + particle filter) is not a possibility, 
# b) we don't have high-quality laser scans so running a SLAM algorithm like
# the gmapping node is pretty useless, and 
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
# 
# subscribes to:
#  /robot_pose_ekf/odom
#
# publishes to:
# /odom
#

import roslib; roslib.load_manifest('robomagellan')
import rospy

import tf

from geometry_msgs.msg import Point, PointStamped, PoseWithCovarianceStamped, Twist, Vector3
from nav_msgs.msg import Odometry

from robomagellan.msg import ConeCaptured

import math

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
        rospy.loginfo("RobomagellanLocalization initialized")

    def publish_localization(self):
        """
            publish the transformation from /map -> /odom
        """
        br = tf.TransformBroadcaster()
        p = self.current_position_offset
        br.sendTransform((p.x, p.y, p.z),
                         self.unit_quaternion,
                         rospy.Time.now(),
                         "odom",
                         "map")

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

    def setup_cone_captured_callback(self):
        def recalculate_drift(data):
            rospy.logwarn("waypoint location: %s" % data.waypoint)
            rospy.logwarn("our current location: %s" % self.odom.pose.pose)

            # calculate the difference between where we think we are and where the cone is
            current_time, x, y, theta = self.current_pose(self.odom)
            xd, yd = data.waypoint.coordinate.x, data.waypoint.coordinate.y
            linear_distance = math.sqrt( (x-xd)*(x-xd) + (y-yd)*(y-yd) )
            rospy.logwarn("(x, y), (xd, yd) = (%.2f, %.2f), (%.2f, %.2f)" % (x, y, xd, yd))

            # take into account the offset between the center of the cone on the center of /base_link
            linear_distance -= 0.125   # about 5 inches, the radius of the cone
            linear_distance -= 0.09    # the distance between /base_laser_link and /base_link
            x_offset, y_offset = -1 * linear_distance * math.cos(theta), -1 * linear_distance * math.sin(theta)

            rospy.logwarn("(x_off, y_off) = (%.2f, %.2f)" % (x_offset, y_offset))

            self.current_position_offset = Point(x_offset, y_offset, 0.0)
            rospy.logwarn("Drift calibration: %s" % self.current_position_offset)
        return recalculate_drift

if __name__ == '__main__':
    rospy.init_node('robomagellan_localization')
    rospy.sleep(3)  # let rxconsole boot up
    rospy.loginfo("Initializing localization node")

    odom_publisher = rospy.Publisher('/odom', Odometry)
    localization = RobomagellanLocalization(odom_publisher)
    rospy.Subscriber('/robot_pose_ekf/odom', PoseWithCovarianceStamped, localization.setup_odom_combined_callback())
    rospy.Subscriber('/cone_captured', ConeCaptured, localization.setup_cone_captured_callback())

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        rate.sleep()
        localization.publish_localization()

