#!/usr/bin/env python

"""

ConeCaptureNavigator and WaypointNavigator, which both inherit from Navigator.

"""

import roslib; roslib.load_manifest('robomagellan')
import rospy

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
from robomagellan.msg import NavigationState

from nav_msgs.msg import Path

import tf
import actionlib

from robomagellan.msg import CaptureWaypointAction
from robomagellan.msg import CaptureWaypointFeedback

import settings

import math

class Navigator():
    def __init__(self, server_name):
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist)
        self.cmd_vel_path_pub = rospy.Publisher('navigator/local_plan', Path)
        self.transformListener = tf.TransformListener()
        self.cone_coord = None
        self.target_coord = None
        self.collided = False
        self.odom = None
        self.state = NavigationState.NONE
        self.server = actionlib.SimpleActionServer(server_name, CaptureWaypointAction, self.execute, False)
        self.server.start()

    def execute(self, goal):
        if not self.responds_to_waypoint(goal.waypoint):
            rospy.logerr("%s does not respond to waypoint of type %s" % (self.__class__, goal.waypoint.type))
            self.server.set_aborted()
            return

        rospy.loginfo("%s Received goal: %s" % (self.__class__, goal))

        rate = rospy.Rate(10.0)
        finished = False
        while not rospy.is_shutdown() and not finished:
            if self.server.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self.__class__)
                self.state = NavigationState.NONE
                self.cone_coord = None
                self.target_coord = None
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
        cmd_vel.linear.x = self.bounded_speed(x)
        cmd_vel.angular.z = self.bounded_turnrate(z)
        self.cmd_vel_pub.publish(cmd_vel)
        self.publish_cmd_vel_path(cmd_vel)

    def publish_cmd_vel_path(self, cmd_vel):
        p = Path()
        p.header.frame_id = "/odom"
        p.header.stamp = rospy.Time.now()

        a = PoseStamped()
        a.pose.position.x = self.odom.pose.pose.position.x
        a.pose.position.y = self.odom.pose.pose.position.y
        p.poses.append(a)

        b = PoseStamped()
        q = self.odom.pose.pose.orientation
        theta = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        theta += 2 * cmd_vel.angular.z
        hyp = cmd_vel.linear.x + cmd_vel.angular.z
        b.pose.position.x = a.pose.position.x + (5 * hyp * math.cos(theta))
        b.pose.position.y = a.pose.position.y + (5 * hyp * math.sin(theta))
        p.poses.append(b)

        self.cmd_vel_path_pub.publish(p)
        return

    def bounded_turnrate(self, turnrate):
        if (turnrate > settings.MAX_TURNRATE):
            return settings.MAX_TURNRATE
        if (turnrate < -1 * settings.MAX_TURNRATE):
            return -1 * settings.MAX_TURNRATE
        return turnrate

    def bounded_speed(self, speed):
        if (speed > 0 and math.fabs(speed) > settings.MAX_VELOCITY):
            return settings.MAX_VELOCITY
        elif (speed < 0 and math.fabs(speed) > settings.MAX_VELOCITY):
            return -1 * settings.MAX_VELOCITY
        elif (speed > 0 and math.fabs(speed) < settings.MIN_VELOCITY):
            return settings.MIN_VELOCITY
        elif (speed < 0 and math.fabs(speed) < settings.MIN_VELOCITY):
            return -1 * settings.MIN_VELOCITY
        return speed

    def target_coord_in_odom_frame(self, goal):
        goal_coord = PointStamped()
        goal_coord.header.frame_id = "/map"
        goal_coord.header.stamp = self.transformListener.getLatestCommonTime("/odom", "/map")
        goal_coord.point = goal.waypoint.coordinate
        return self.transformListener.transformPoint("/odom", goal_coord)

    def rotate_towards_goal(self, goal):
        if self.target_coord == None:
            # set up our target the first time through
            self.target_coord = self.target_coord_in_odom_frame(goal)

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

        if (math.fabs(terr) > math.pi):
            # rotate in the other direction for efficiency 
            terr -= math.pi
            terr *= -1

        if (math.fabs(terr) < settings.THETA_TOLERANCE):
            # we've reached the desired orientation, time to move towards our goal
            self.target_coord = None
            self.state = NavigationState.MOVE_TOWARDS_GOAL
        else:
            # need to keep turning to reach our desired orientation
            turnrate = settings.A2*terr
            if (turnrate > 0 and math.fabs(turnrate) < settings.MIN_TURNRATE):
                turnrate = settings.MIN_TURNRATE
            elif (turnrate < 0 and math.fabs(turnrate) < settings.MIN_TURNRATE):
                turnrate = -1 * settings.MIN_TURNRATE
            self.publish_cmd_vel(0.0, turnrate)


class WaypointNavigator(Navigator):
    def __init__(self, server_name):
        Navigator.__init__(self, server_name)

    def responds_to_waypoint(self, waypoint):
        # Waypoint navigator can move towards cones as well, 
        # until we get in range of camera sensors
        return True

    def capture_waypoint(self, goal):

        if self.odom is None:
            rospy.logwarn("Waiting for initial robot odometry!")
            return False

        if self.state == NavigationState.NONE:
            self.state = NavigationState.ROTATE_INTO_POSITION

        elif self.state == NavigationState.ROTATE_INTO_POSITION:
            self.rotate_towards_goal(goal)

        elif self.state == NavigationState.MOVE_TOWARDS_GOAL:
            if self.collided:
                rospy.logwarn("Oops, we hit something!")
                self.state == NavigationState.AVOID_OBSTACLE
            else:
                return self.move_towards_waypoint(goal)

        elif self.state == NavigationState.ROTATE_TO_FIND_PATH:
            self.state = NavigationState.MOVE_TOWARDS_GOAL

        elif self.state == NavigationState.AVOID_OBSTACLE:
            # TODO implement 
            # until then... full stop
            self.publish_cmd_vel(0.0, 0.0)

        # haven't reached our goal yet...
        return False

    def move_towards_waypoint(self, goal):
        """
            moves the robot towards the goal, with linear-x and
            angular-z twist commands proportional to the error in
            the linear and y-direction, respectively.
        """

        # current pose
        x = self.odom.pose.pose.position.x
        y = self.odom.pose.pose.position.y

        if self.target_coord == None:
            # set up our target the first time through
            self.target_coord = self.target_coord_in_odom_frame(goal)
            self.xinit = x
            self.yinit = y

        # desired pose
        xd = self.target_coord.point.x
        yd = self.target_coord.point.y

        # xerr = error in the linear direction
        xerr = math.sqrt( (x-xd)*(x-xd) + (y-yd)*(y-yd) )

        # yerr = error in the longitudinal direction
        #
        # perpendicular distance from line connecting initial pos and desired pos
        #
        # if the line going from initial point to desired point is defined as
        #   Ax + By + C = 0
        # or in another form (where m is the slope of the line)
        #   y - yd = m(x - xd)
        # then the distance is from point (m,n) to the line is (http://tinyurl.com/43s7mur)
        #   (Am + Bn + C)/sqrt(A*A+B*B)
        # where
        #   A = -m
        #   B = 1
        #   C = m*xd - yd
        #
        slope = (yd - self.yinit)/(xd - self.xinit)
        A = -slope
        B = 1
        C = slope * xd - yd
        yerr = (A*x + B*y + C)/(math.sqrt(A*A+B*B))
        # need to do this because of how we defined theta
        if (xd > x):
            yerr *= -1 

        if (math.fabs(xerr) < settings.WAYPOINT_THRESHOLD and 
            math.fabs(yerr) < settings.WAYPOINT_THRESHOLD):
            # we've reached our waypoint! time to return success
            self.state = NavigationState.NONE
            self.target_coord = None
            self.server.set_succeeded()
            return True

        else:
            # need to continue to move towards waypoint

            speed = settings.LAMBDA * xerr
            turnrate = settings.A1*yerr

            self.publish_cmd_vel(speed, turnrate)
            return False



class ConeCaptureNavigator(Navigator):
    def __init__(self, server_name):
        Navigator.__init__(self, server_name)

    def responds_to_waypoint(self, waypoint):
        return waypoint.type == 'C'

    def capture_waypoint(self, goal):

        if self.odom is None:
            rospy.logwarn("Waiting for initial robot odometry!")
            return False

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

            self.flush_outdated_cone_coord_data()

            if self.collided:
                # we're done, time to return the action service
                rospy.loginfo("cone captured!")
                self.move_backwards_to_clear_cone()
                self.state = NavigationState.NONE
                self.cone_coord = None
                self.target_coord = None
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
        self.publish_cmd_vel(settings.MIN_VELOCITY, z)

    def move_backwards_to_clear_cone(self):
        rospy.loginfo("moving backwards to clear cone")

        # first, make sure we're stopped
        for i in range(10):
            self.publish_cmd_vel(0.0, 0.0)
            rospy.sleep(.1)

        # now, move backwards
        for i in range(20):
            linear_x = -1 * settings.MIN_VELOCITY
            self.publish_cmd_vel(linear_x, 0.0)
            rospy.sleep(.1)

        # stop again before continuing
        self.publish_cmd_vel(0.0, 0.0)

    def flush_outdated_cone_coord_data(self):
        if self.cone_coord is None:
            return
        
        now = rospy.Time.now().to_sec()
        latency = now - self.cone_coord.header.stamp.to_sec()
        if latency > 0.5:
            self.cone_coord = None

    def rotate_in_place_to_find_cone(self):
        if self.cone_coord != None:
            # we found the cone!
            self.state = NavigationState.CAPTURE_CONE
        else:
            # rotate in place until the cone comes into view
            # TODO abort if we can't find the cone after some amount of time
            self.publish_cmd_vel(0.0, settings.MIN_TURNRATE)

