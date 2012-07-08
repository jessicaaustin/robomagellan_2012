#!/usr/bin/env python

"""

strategy
- determines where to move the robot next
- use move_base to move the robot towards point waypoints, and the cone_capture
  node to move the robot towards cone waypoints

listens to:
 /cone_coord

publishes to:
 /move_base/goal
 /capture_cone/goal
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
from geometry_msgs.msg import Quaternion
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from move_base_msgs.msg import MoveBaseAction
from move_base_msgs.msg import MoveBaseGoal
from robomagellan.msg import CaptureConeAction
from robomagellan.msg import CaptureConeGoal
from actionlib_msgs.msg import GoalStatus
from topic_tools.srv import MuxSelect

import settings

import os
import sys

class Strategizer():
    def __init__(self, waypoints_file):
        rospy.loginfo("Initializing Strategizer")
        self.cone_capture_mode = None
        self.transformListener = tf.TransformListener()
        self.load_waypoints()
        self.setup_goal_actions()

    def load_waypoints(self):        
        waypoint_file_reader = WaypointFileReader()
        self.waypoints = waypoint_file_reader.read_file(waypoints_file)
        if len(self.waypoints) <= 0:
            raise ROSInitException("No waypoints given!")
        rospy.loginfo("Strategizer loaded with waypoints:\n %s" % self.waypoints)

    def setup_goal_actions(self):
        rospy.loginfo("Waiting for required services...")
        self.move_base_client = SimpleActionClient('move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()
        rospy.loginfo("move_base action client loaded")
        self.capture_cone_client = SimpleActionClient('capture_cone', CaptureConeAction)
        self.capture_cone_client.wait_for_server()
        rospy.loginfo("capture_cone action client loaded")
        rospy.wait_for_service('mux_cmd_vel/select')
        rospy.loginfo("mux_cmd_vel client loaded")

        # send the first goal
        rospy.loginfo("Strategizer sending initial goal")
        self.current_waypoint_idx = 0
        self.send_next_move_base_goal()

    # monitor the cone_coord topic, and if we are attempting a cone waypoint
    # and it is sufficiently close, switch to cone capture mode
    def setup_cone_coord_callback(self):
        def cone_coord_callback(data):
            cone_coord = self.transformListener.transformPoint("base_link", data)
            distance_to_cone = cone_coord.point.x
            if not self.cone_capture_mode and \
               self.waypoints[self.current_waypoint_idx].type == 'C' and \
               distance_to_cone < settings.MAX_DISTANCE_TO_CAPTURE:
                self.switch_to_cone_capture_mode()
        return cone_coord_callback

    def send_next_move_base_goal(self):
        rospy.loginfo("Moving towards next goal:\n %s" % self.waypoints[self.current_waypoint_idx])
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "/odom"
        goal.target_pose.pose.position = self.waypoints[self.current_waypoint_idx].coordinate
        # base_local_planner is configured to ignore goal yaw, so goal orientation doesn't actually matter
        goal.target_pose.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
        self.move_base_client.send_goal(goal, self.receive_move_base_goal_done_callback)

    def receive_move_base_goal_done_callback(self, status, result):
        rospy.loginfo("move_base returned with status: %s" % status)
        if status == GoalStatus.PREEMPTED:
            # we cancelled the goal, so capture_cone should be taking care of things... do nothing
            return

        if status != GoalStatus.SUCCEEDED:
            rospy.logerr("move_base failed to achieve goal! result = %s" % result)
            rospy.loginfo("skipping this waypoint...")

        self.current_waypoint_idx = self.current_waypoint_idx + 1
        if self.current_waypoint_idx == len(self.waypoints):
            rospy.loginfo("All waypoints reached! Finished strategizing.")
        else:
            self.send_next_move_base_goal()

    def switch_to_cone_capture_mode(self):
        rospy.loginfo("Switching to cone capture mode!")

        # cancel the move_base goal
        self.move_base_client.cancel_goal()
        rospy.loginfo("move_base goal cancelled")

        # switch to cone_capture node for publishing to /cmd_vel
        self.cone_capture_mode = True
        self.switch_cmd_vel("cmd_vel_capture_cone")

        # send the goal to capture_cone node
        waypoint_to_capture = self.waypoints[self.current_waypoint_idx]
        rospy.loginfo("Attempting to capture cone at:\n %s" % waypoint_to_capture)
        goal = CaptureConeGoal()
        goal.waypoint = waypoint_to_capture
        self.capture_cone_client.send_goal(goal, self.receive_capture_cone_done_callback)

    def receive_capture_cone_done_callback(self, status, result):
        rospy.loginfo("capture_cone returned with status: %s" % status)
        if status != GoalStatus.SUCCEEDED:
            rospy.logerr("capture_cone failed to achieve goal! result = %s" % result)
            rospy.loginfo("skipping this waypoint...")
        self.current_waypoint_idx = self.current_waypoint_idx + 1
        if self.current_waypoint_idx == len(self.waypoints):
            rospy.loginfo("All waypoints reached! Finished strategizing.")
        else:
            # switch back to move_base for next waypoint
            self.switch_cmd_vel("cmd_vel_move_base")
            self.cone_capture_mode = False
            self.send_next_move_base_goal()

    def switch_cmd_vel(self, topic):
        rospy.loginfo("switching cmd_vel_mux to %s" % topic)
        cmd_vel_mux_select = rospy.ServiceProxy('mux_cmd_vel/select', MuxSelect)
        resp = cmd_vel_mux_select(topic)
        rospy.loginfo("result of switching cmd_vel_mux to %s: %s" % (topic, resp))


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
            
