#!/usr/bin/env python

"""

strategy
- determines where to move the robot next
- switches between the navigator_capture_waypoint and navigator_capture_cone navigators
  depending on the type of waypoint we are currently trying to reach

listens to:
 /cone_coord

publishes to:
 /capture_cone/goal
 /capture_waypoint/goal
 /mux_cmd_vel/select

"""

import roslib; roslib.load_manifest('robomagellan')
import rospy

from actionlib import SimpleActionClient
from rospy.exceptions import ROSInitException

import tf

from waypoint_reader import WaypointFileReader

from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from robomagellan.msg import CaptureWaypointAction
from robomagellan.msg import CaptureWaypointGoal
from actionlib_msgs.msg import GoalStatus
from topic_tools.srv import MuxSelect

import settings

import os
import sys
import math

class Strategizer():
    def __init__(self, waypoints_file):
        rospy.loginfo("Initializing Strategizer")
        self.cone_coord = None
        self.cone_capture_mode = False
        self.transformListener = tf.TransformListener()
        self.load_waypoints()
        self.setup_goal_actions()
        self.wait_for_odom()
        self.send_first_goal()

    def load_waypoints(self):        
        waypoint_file_reader = WaypointFileReader()
        self.waypoints = waypoint_file_reader.read_file(waypoints_file)
        if len(self.waypoints) <= 0:
            raise ROSInitException("No waypoints given!")
        rospy.loginfo("Strategizer loaded with waypoints:\n %s" % self.waypoints)

    def setup_goal_actions(self):
        rospy.logwarn("Waiting for required services...")
        self.capture_cone_client = SimpleActionClient('capture_cone', CaptureWaypointAction)
        self.capture_cone_client.wait_for_server()
        rospy.loginfo("capture_cone action client loaded")
        self.capture_waypoint_client = SimpleActionClient('capture_waypoint', CaptureWaypointAction)
        self.capture_waypoint_client.wait_for_server()
        rospy.loginfo("capture_waypoint action client loaded")
        rospy.wait_for_service('mux_cmd_vel/select')
        rospy.loginfo("mux_cmd_vel client loaded")
        rospy.logwarn("Services loaded")

    def wait_for_odom(self):
        self.odom = None
        rospy.Subscriber('odom', Odometry, self.setup_odom_callback())
        while self.odom is None:
            rospy.logwarn("Waiting for initial odometry...")
        rospy.logwarn("Initial odom loaded! Sleeping for 3 sec...")
        rospy.sleep(3)

    def setup_odom_callback(self):
        def odom_callback(data):
            self.odom = data
        return odom_callback

    def send_first_goal(self):
        rospy.loginfo("Strategizer sending initial goal")
        self.current_waypoint_idx = 0
        self.send_next_capture_waypoint_goal()

    # monitor the cone_coord topic
    def setup_cone_coord_callback(self):
        def cone_coord_callback(data):
            self.cone_coord = data
        return cone_coord_callback

    # check if next waypoint is a cone, and we are sufficiently close
    # to switch to cone tracking mode
    def check_for_cone(self):
        if self.current_waypoint_idx == len(self.waypoints):
            rospy.logwarn("All waypoints reached! Finished strategizing.")
            return

        self.flush_outdated_cone_coord_data()

        # if we're already in cone-capture mode, or
        # the current waypoint is not a cone... continue
        if self.cone_capture_mode or \
           self.waypoints[self.current_waypoint_idx].type != 'C':
            return

        # if we are sufficiently close, switch to cone capture mode
        cone_waypoint = self.waypoints[self.current_waypoint_idx]
        cone_coord_in_base_link_frame = self.waypoint_in_base_link_frame(cone_waypoint)
        (x,y) = cone_coord_in_base_link_frame.point.x, cone_coord_in_base_link_frame.point.y
        distance_to_cone = math.sqrt(x*x + y*y)
        if (distance_to_cone < settings.DISTANCE_TO_CAPTURE):
            if self.cone_coord:
                self.switch_to_cone_capture_mode()
            else:
                rospy.logwarn("Should be close enough to the cone, but no visual yet! Moving closer...")
        if (self.cone_coord is None and distance_to_cone < settings.DISTANCE_TO_CAPTURE_NO_VISUAL):
            rospy.logwarn("We're within %f m of the cone, and still have no visual. Switching to cone capture mode anyway." % settings.DISTANCE_TO_CAPTURE_NO_VISUAL)
            self.switch_to_cone_capture_mode()
            

    def waypoint_in_base_link_frame(self, waypoint_msg):
        waypoint = PointStamped()
        waypoint.header.frame_id = "/map"
        waypoint.header.stamp = self.transformListener.getLatestCommonTime("/base_link", "/map")
        waypoint.point = waypoint_msg.coordinate
        return self.transformListener.transformPoint("/base_link", waypoint)

    def flush_outdated_cone_coord_data(self):
        if self.cone_coord is None:
            return
        now = rospy.Time.now().to_sec()
        latency = now - self.cone_coord.header.stamp.to_sec()
        if latency > 0.5:
            self.cone_coord = None

    def send_next_capture_waypoint_goal(self):
        waypoint_to_capture = self.waypoints[self.current_waypoint_idx]
        rospy.logwarn("Moving towards next goal:\n %s" % waypoint_to_capture)
        goal = CaptureWaypointGoal()
        goal.waypoint = waypoint_to_capture
        self.capture_waypoint_client.send_goal(goal, self.recieve_capture_waypoint_done_callback)

    def recieve_capture_waypoint_done_callback(self, status, result):
        rospy.logwarn("capture_waypoint returned with status: %s" % status)
        if status == GoalStatus.PREEMPTED:
            # we cancelled the goal, so capture_cone should be taking care of things... do nothing
            return

        if status != GoalStatus.SUCCEEDED:
            rospy.logerr("capture_waypoint failed to achieve goal! result = %s" % result)
            rospy.logwarn("skipping this waypoint...")

        self.current_waypoint_idx = self.current_waypoint_idx + 1
        if self.current_waypoint_idx == len(self.waypoints):
            rospy.logwarn("All waypoints reached! Finished strategizing.")
        else:
            self.send_next_capture_waypoint_goal()

    def switch_to_cone_capture_mode(self):
        rospy.logwarn("Switching to cone capture mode!")

        # cancel the capture_waypoint goal
        self.capture_waypoint_client.cancel_goal()
        rospy.loginfo("capture_waypoint goal cancelled")

        # switch to cone_capture node for publishing to /cmd_vel
        self.cone_capture_mode = True
        self.switch_cmd_vel("cmd_vel_capture_cone")

        # send the goal to capture_cone node
        waypoint_to_capture = self.waypoints[self.current_waypoint_idx]
        rospy.logwarn("Attempting to capture cone at:\n %s" % waypoint_to_capture)
        goal = CaptureWaypointGoal()
        goal.waypoint = waypoint_to_capture
        self.capture_cone_client.send_goal(goal, self.receive_capture_cone_done_callback)

    def receive_capture_cone_done_callback(self, status, result):
        rospy.logwarn("capture_cone returned with status: %s" % status)
        if status != GoalStatus.SUCCEEDED:
            rospy.logerr("capture_cone failed to achieve goal! result = %s" % result)
            rospy.logwarn("skipping this waypoint...")
        self.current_waypoint_idx = self.current_waypoint_idx + 1
        if self.current_waypoint_idx == len(self.waypoints):
            rospy.logwarn("All waypoints reached! Finished strategizing.")
        else:
            # switch back to capture_waypoint for next waypoint
            self.switch_cmd_vel("cmd_vel_capture_waypoint")
            self.cone_capture_mode = False
            self.send_next_capture_waypoint_goal()

    def switch_cmd_vel(self, topic):
        rospy.loginfo("switching cmd_vel_mux to %s" % topic)
        cmd_vel_mux_select = rospy.ServiceProxy('mux_cmd_vel/select', MuxSelect)
        resp = cmd_vel_mux_select(topic)
        rospy.loginfo("result of switching cmd_vel_mux to %s: %s" % (topic, resp))


    def publish_waypoint_markers(self):
        pub_waypoint_markers = rospy.Publisher('waypoint_markers', MarkerArray)
        marker_array = MarkerArray()
        for i in range(len(self.waypoints)):
            waypoint = self.waypoints[i]
            waypoint_marker = Marker()
            waypoint_marker.id = i
            waypoint_marker.header.frame_id = "/map"
            waypoint_marker.header.stamp = rospy.Time.now()
            if (waypoint.type == 'P'):
                waypoint_marker.type = 5  # Line List
                waypoint_marker.text = 'waypoint_%s_point' % i
                waypoint_marker.color.r = 176.0
                waypoint_marker.color.g = 224.0
                waypoint_marker.color.b = 230.0
                waypoint_marker.color.a = 0.5
                waypoint_marker.scale.x = 0.5
                c = waypoint.coordinate
                waypoint_marker.points.append(Point(c.x, c.y, c.z))
                waypoint_marker.points.append(Point(c.x, c.y, c.z + 3.0))
            else:
                waypoint_marker.type = 3  # Cylinder
                waypoint_marker.text = 'waypoint_%s_cone' % i
                waypoint_marker.color.r = 255.0
                waypoint_marker.color.g = 69.0
                waypoint_marker.color.b = 0.0
                waypoint_marker.color.a = 1.0
                waypoint_marker.scale.x = 0.3
                waypoint_marker.scale.y = 0.3
                waypoint_marker.scale.z = 0.5
                waypoint_marker.pose.position = waypoint.coordinate
            marker_array.markers.append(waypoint_marker)

        if self.current_waypoint_idx != len(self.waypoints):
            current_waypoint_marker = Marker()
            current_waypoint_marker.id = 999
            current_waypoint_marker.header.frame_id = "/map"
            current_waypoint_marker.header.stamp = rospy.Time.now()
            current_waypoint_marker.type = 2  # Sphere
            current_waypoint_marker.text = 'current_waypoint'
            current_waypoint_marker.color.r = 255.0
            current_waypoint_marker.color.g = 0.0
            current_waypoint_marker.color.b = 0.0
            current_waypoint_marker.color.a = 1.0
            current_waypoint_marker.scale.x = 0.3
            current_waypoint_marker.scale.y = 0.3
            current_waypoint_marker.scale.z = 0.3
            current_waypoint = self.waypoints[self.current_waypoint_idx]
            current_waypoint_marker.pose.position.x = current_waypoint.coordinate.x
            current_waypoint_marker.pose.position.y = current_waypoint.coordinate.y
            current_waypoint_marker.pose.position.z = 1.0
            marker_array.markers.append(current_waypoint_marker)
 
        pub_waypoint_markers.publish(marker_array)



if __name__ == '__main__':
    rospy.init_node('strategy')
    rospy.sleep(2)  # let rxconsole boot up
    rospy.loginfo("Initializing strategy node")

    if len(sys.argv) < 2 or not os.path.exists(sys.argv[1]) or not os.path.isfile(sys.argv[1]):
        rospy.logerr("Waypoint file %s not found" % (sys.argv[1]))
        sys.exit(1)
    else:
        waypoints_file = sys.argv[1]
        strategizer = Strategizer(waypoints_file)
        rospy.Subscriber('cone_coord', PointStamped, strategizer.setup_cone_coord_callback())

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        rate.sleep()
        strategizer.check_for_cone()
        strategizer.publish_waypoint_markers()
            
