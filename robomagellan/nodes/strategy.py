#!/usr/bin/env python

"""

strategy
- determines where to move the robot next
- uses the move_base ROS package to actually move the robot to the goal

listens to:
 /cone_coord

publishes to:
 /move_base/goal

"""

import roslib; roslib.load_manifest('robomagellan')
import rospy

from actionlib import SimpleActionClient
from rospy.exceptions import ROSInitException

from waypoint_reader import WaypointFileReader

from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Quaternion
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from move_base_msgs.msg import MoveBaseAction
from move_base_msgs.msg import MoveBaseGoal
from actionlib_msgs.msg import GoalStatus

import os
import sys

class Strategizer():
    def __init__(self, waypoints_file):
        self.cone_coord = None
        self.load_waypoints()
        self.setup_goal_actions()

    def load_waypoints(self):        
        waypoint_file_reader = WaypointFileReader()
        self.waypoints = waypoint_file_reader.read_file(waypoints_file)
        if len(self.waypoints) <= 0:
            raise ROSInitException("No waypoints given!")
        rospy.loginfo("Strategizer loaded with waypoints:\n %s" % self.waypoints)

    def setup_goal_actions(self):
        self.move_base_client = SimpleActionClient('move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()
        # send the first goal
        rospy.loginfo("Strategizer sending initial goal")
        self.current_waypoint_idx = 0
        self.send_next_goal()

    def setup_cone_coord_callback(self):
        def cone_coord_callback(data):
            self.cone_coord = data.point
        return cone_coord_callback

    # TODO change the color of the waypoint depending on whether it is 
    # the next goal, has already been achieved, was skipped, etc
    def publish_waypoint_markers(self):
        pub_waypoint_markers = rospy.Publisher('waypoint_markers', MarkerArray)
        marker_array = MarkerArray()
        for i in range(len(self.waypoints)):
            waypoint = self.waypoints[i]
            waypoint_marker = Marker()
            waypoint_marker.id = i
            waypoint_marker.header.frame_id = "/odom"
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
        pub_waypoint_markers.publish(marker_array)

    def send_next_goal(self):
        rospy.loginfo("Moving towards next goal:\n %s" % self.waypoints[self.current_waypoint_idx])
        goal = MoveBaseGoal()
        goal.target_pose
        goal.target_pose.header.frame_id = "/odom"
        goal.target_pose.pose.position = self.waypoints[self.current_waypoint_idx].coordinate
        # we don't care about orientation currently, so we'll just send something reasonable
        # TODO change config for move_base so that it actually doesn't care about goal orientation
        goal.target_pose.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
        self.move_base_client.send_goal(goal, self.receive_goal_done_callback)

    def receive_goal_done_callback(self, status, result):
        rospy.loginfo("move_base returned with status: %s" % status)
        if status != GoalStatus.SUCCEEDED:
            rospy.logerr("move_base failed to achieve goal! result = %s" % result)
            rospy.loginfo("skipping this waypoint...")
        self.current_waypoint_idx = self.current_waypoint_idx + 1
        if self.current_waypoint_idx == len(self.waypoints):
            rospy.loginfo("All waypoints reached! Finished strategizing.")
        else:
            self.send_next_goal()


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
        rospy.Subscriber('cone_coord', PointStamped, strategizer.setup_cone_coord_callback())

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        rate.sleep()
        strategizer.publish_waypoint_markers()
            
