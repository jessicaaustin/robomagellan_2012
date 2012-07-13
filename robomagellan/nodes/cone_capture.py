#!/usr/bin/env python

"""

cone_capture
  based on the location of a cone published by the cone_tracker node,
  this node will send commands to the robot base on the /cmd_vel topic,
  as it attempts to reach and physically contact the cone.

implements actionlib for:
 /capture_cone

listens to:
 /cone_coord
 /collision

publishes to:
 /cmd_vel

"""

import roslib; roslib.load_manifest('robomagellan')
import rospy

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped
from robomagellan.msg import BoolStamped
from robomagellan.msg import NavigationState
from nav_msgs.msg import Odometry

import tf
import actionlib

from robomagellan.msg import CaptureWaypointAction
from robomagellan.msg import CaptureWaypointFeedback

import settings

import math

class Navigator():
    def __init__(self, publisher):
        self.publisher = publisher
        self.transformListener = tf.TransformListener()
        self.cone_coord = None
        self.target_coord = None
        self.collided = False
        self.odom = None
        self.state = NavigationState.NONE
        self.server = actionlib.SimpleActionServer('capture_cone', CaptureWaypointAction, self.execute, False)
        self.server.start()

    def execute(self, goal):
        if not self.responds_to_waypoint(goal.waypoint):
            rospy.loginfo("%s does not respond to waypoint of type %s" % (self.__class__, goal.waypoint.type))
            return

        rospy.loginfo("%s Received goal: %s" % (self.__class__, goal))

        rate = rospy.Rate(10.0)
        finished = False
        while not rospy.is_shutdown() and not finished:
            if self.server.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self.__class__)
                self.server.set_preempted()
                finished = True
            else:
                finished = self.capture_waypoint(goal)
                self.publish_feedback()
            rate.sleep()

    def setup_cone_coord_callback(self):
        def cone_coord_callback(data):
            # only save this data if we are moving towards a cone 
            # we do this to avoid outdated cone data
            if self.state == NavigationState.CAPTURE_CONE or self.state == NavigationState.ROTATE_TO_FIND_PATH:
                self.cone_coord = data
            else:
                self.cone_coord = None
        return cone_coord_callback

    def setup_collision_callback(self):
        def collision_callback(data):
            self.collided = data.value
        return collision_callback

    def setup_odom_callback(self):
        def odom_callback(data):
            self.odom = data
        return odom_callback

    def publish_feedback(self):
        feedback = CaptureWaypointFeedback()
        feedback.status.state = self.state
        feedback.status.text = self.status_message()
        self.server.publish_feedback(feedback)

    def status_message(self):
        if self.state == NavigationState.ROTATE_INTO_POSITION:
            return "ROTATE_INTO_POSITION"
        elif self.state == NavigationState.MOVE_TOWARDS_GOAL:
            return "MOVE_TOWARDS_GOAL"
        elif self.state == NavigationState.CAPTURE_CONE:
            return "CAPTURE_CONE"
        elif self.state == NavigationState.ROTATE_TO_FIND_PATH:
            return "ROTATE_TO_FIND_PATH"
        elif self.state == NavigationState.AVOID_OBSTACLE:
            return "AVOID_OBSTACLE"
        else:
            return ""
       
    def publish_cmd_vel(self, x, z):
        cmd_vel = Twist()
        cmd_vel.linear.x = x
        cmd_vel.angular.z = z
        self.publisher.publish(cmd_vel)

    def rotate_towards_goal(self, goal):
        if self.target_coord == None:
            # set up our target the first time through
            goal_coord = PointStamped()
            goal_coord.header.frame_id = "/map"
            goal_coord.header.stamp = self.transformListener.getLatestCommonTime("/odom", "/map")
            goal_coord.point = goal.waypoint.coordinate
            self.target_coord = self.transformListener.transformPoint("/odom", goal_coord)

        # current pose
        x = self.odom.pose.pose.position.x
        y = self.odom.pose.pose.position.y
        q = self.odom.pose.pose.orientation
        # desired pose
        xd = self.target_coord.point.x
        yd = self.target_coord.point.y
        td = math.atan2(yd-y, xd-x)
        # difference between these two poses
        theta = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        terr = td - theta

        if (math.fabs(terr) < settings.THETA_TOLERANCE):
            # we've reached the desired orientation, time to move towards our goal
            self.target_coord = None
            self.state = NavigationState.MOVE_TOWARDS_GOAL
        else:
            # need to keep turning to reach our desired orientation
            turnrate = settings.A2*terr
            self.publish_cmd_vel(0.0, turnrate)


class ConeCaptureNavigator(Navigator):
    def __init__(self, publisher):
        Navigator.__init__(self, publisher)

    def responds_to_waypoint(self, waypoint):
        return waypoint.type == 'C'

    def capture_waypoint(self, goal):

        if self.state == NavigationState.NONE:
            self.state = NavigationState.ROTATE_INTO_POSITION

        elif self.state == NavigationState.ROTATE_INTO_POSITION:
            self.rotate_towards_goal(goal)

        elif self.state == NavigationState.MOVE_TOWARDS_GOAL:
            # Assumption here... the intermediate waypoints are laid out in such a way
            # that by the time a cone waypoint is sent, we are close enough to the cone
            # that we should be able to see it
            self.state = NavigationState.CAPTURE_CONE

        elif self.state == NavigationState.CAPTURE_CONE:
            if self.collided:
                # we're done, time to return the action service
                rospy.loginfo("cone captured!")
                self.move_backwards_to_clear_cone()
                self.state = NavigationState.NONE
                self.cone_coord = None
                self.server.set_succeeded()
                return True
            elif self.cone_coord == None:
                rospy.logwarn("Attempting to capture cone but no cone_coord available!")
                self.state = NavigationState.ROTATE_TO_FIND_PATH
            else:
                self.move_towards_cone()
            # TODO add check for obstacles...

        elif self.state == NavigationState.ROTATE_TO_FIND_PATH:
            self.rotate_in_place_to_find_cone()

        # haven't reached our goal yet...
        return False

    def move_towards_cone(self):
        """
            moves the robot forward at a constant velocity, 
            with yaw commands proportional to the error in the 
            y-direction
        """
        z = self.cone_coord.point.y
        if z > 0.2:
            z = 0.2
        if z < -0.2:
            z = -0.2
        self.publish_cmd_vel(settings.SPEED_TO_CAPTURE, z)

    def move_backwards_to_clear_cone(self):
        rospy.loginfo("moving backwards to clear cone")

        # first, make sure we're stopped
        for i in range(10):
            cmd_vel = Twist()
            self.publisher.publish(cmd_vel)
            rospy.sleep(.1)

        # now, move backwards
        for i in range(20):
            cmd_vel = Twist()
            cmd_vel.linear.x = -1 * settings.SPEED_TO_CAPTURE
            self.publisher.publish(cmd_vel)
            rospy.sleep(.1)

        # stop again before continuing
        cmd_vel = Twist()
        self.publisher.publish(cmd_vel)

    def rotate_in_place_to_find_cone(self):
        if self.cone_coord != None:
            # we found the cone!
            self.state = NavigationState.CAPTURE_CONE
        else:
            # rotate in place until the cone comes into view
            # TODO abort if we can't find the cone after some amount of time
            self.publish_cmd_vel(0.0, settings.SPEED_TO_ROTATE)


if __name__ == '__main__':
    rospy.init_node('cone_capture')
    rospy.sleep(3)  # let rxconsole boot up
    rospy.loginfo("Initializing cone_capture node")

    publisher = rospy.Publisher('cmd_vel', Twist)
    capturer = ConeCaptureNavigator(publisher)
    rospy.Subscriber('cone_coord', PointStamped, capturer.setup_cone_coord_callback())
    rospy.Subscriber('collision', BoolStamped, capturer.setup_collision_callback())
    rospy.Subscriber('odom', Odometry, capturer.setup_odom_callback())
            
    rospy.spin()
