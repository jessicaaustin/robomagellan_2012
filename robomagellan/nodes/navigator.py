#!/usr/bin/env python

"""

ConeCaptureNavigator and WaypointNavigator, which both inherit from Navigator.

subscribes to:

publishes to:
 /cmd_vel
 /navigator/local_plan  (for visualization)
 /cone_captured

"""

import roslib; roslib.load_manifest('robomagellan')
import rospy

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
from robomagellan.msg import NavigationState
from robomagellan.msg import ConeCaptured

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
        self.obstacle_detected = False
        self.odom = None
        self.turning_direction = None
        self.yerr_accumulated = 0
        self.yerr_previous = 0
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
                self.reset_everything()
                self.server.set_preempted()
                finished = True
            else:
                finished = self.capture_waypoint(goal)
                self.publish_feedback()
            rate.sleep()

    def setup_cone_coord_callback(self):
        def cone_coord_callback(data):
            self.cone_coord = data
        return cone_coord_callback

    def setup_collision_callback(self):
        def collision_callback(data):
            self.collided = data.value
        return collision_callback

    def setup_obstacle_callback(self):
        def obstacle_callback(data):
            self.obstacle_detected = data.value
        return obstacle_callback

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
       
    def publish_cmd_vel(self, x, z, source):
        cmd_vel = Twist()
        cmd_vel.linear.x = self.bounded_speed(x)
        cmd_vel.angular.z = self.bounded_turnrate(z)
        rospy.loginfo("publishing cmd_vel from [%s] (%.2f, %.2f)" % (source, cmd_vel.linear.x, cmd_vel.angular.z))
        self.cmd_vel_pub.publish(cmd_vel)
        self.publish_cmd_vel_path(cmd_vel)

    def full_stop(self):
        rospy.logwarn("FULL STOP")
        cmd_vel = Twist()
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
        if turnrate == 0.0:
            return 0.0
        if (turnrate > settings.MAX_TURNRATE):
            return settings.MAX_TURNRATE
        if (turnrate < -1 * settings.MAX_TURNRATE):
            return -1 * settings.MAX_TURNRATE
        return turnrate

    def bounded_speed(self, speed):
        if speed == 0.0:
            return 0.0
        if (speed > 0 and math.fabs(speed) > settings.MAX_VELOCITY):
            return settings.MAX_VELOCITY
        elif (speed < 0 and math.fabs(speed) > settings.MAX_VELOCITY):
            return -1 * settings.MAX_VELOCITY
        elif (speed > 0 and math.fabs(speed) < settings.MIN_VELOCITY):
            return settings.MIN_VELOCITY
        elif (speed < 0 and math.fabs(speed) < settings.MIN_VELOCITY):
            return -1 * settings.MIN_VELOCITY
        return speed

    def stop_and_sleep(self, sec):
        self.full_stop()
        rospy.sleep(sec)

    def target_coord_in_odom_frame(self, goal):
        goal_coord = PointStamped()
        goal_coord.header.frame_id = "/map"
        goal_coord.header.stamp = self.transformListener.getLatestCommonTime("/odom", "/map")
        goal_coord.point = goal.waypoint.coordinate
        return self.transformListener.transformPoint("/odom", goal_coord)

    def rotate_towards_goal(self, goal):
        rospy.loginfo("Rotating towards goal")
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
            self.stop_and_sleep(0.5)
            self.turning_direction = None
            self.state = NavigationState.MOVE_TOWARDS_GOAL
        else:
            # need to keep turning to reach our desired orientation
            rospy.logwarn("terr=%.2f, should be less than %.2f" % (terr, settings.THETA_TOLERANCE)) 
            if self.turning_direction is None:
                if terr > 0:
                    self.turning_direction = 'left'
                else:
                    self.turning_direction = 'right'
            if self.turning_direction == 'left':
                rospy.logwarn("turning left to adjust orientation")
                self.turn_left_to_adjust_orientation()
                self.stop_and_sleep(1)
            else:
                rospy.logwarn("turning right to adjust orientation")
                self.turn_right_to_adjust_orientation()
                self.stop_and_sleep(1)
            
    def turn_left_to_adjust_orientation(self):
        self.turn_left(settings.ROTATE_CYCLES, settings.ROTATE_VEL, 'turn_left_to_adjust_orientation')

    def turn_right_to_adjust_orientation(self):
        self.turn_right(settings.ROTATE_CYCLES, settings.ROTATE_VEL, 'turn_right_to_adjust_orientation')

    def turn_to_avoid_obstacle(self):
        self.turn_right(settings.CONE_SEARCH_ROT_TIME, settings.CONE_SEARCH_ROT_VEL, 'turn_to_avoid_obstacle')

    def move_backwards(self):
        rospy.loginfo("moving backwards to clear obstacle")

        # first, make sure we're stopped
        for i in range(10):
            self.stop_and_sleep(.1)

        # now, move backwards
        for i in range(20):
            linear_x = -1 * settings.MIN_VELOCITY
            self.publish_cmd_vel(linear_x, 0.0, 'move_backwards')
            rospy.sleep(.1)

        # stop again before continuing
        self.full_stop()

    def turn_left(self, cycles, velocity, description):
        self.turn(cycles, velocity, description)

    def turn_right(self, cycles, velocity, description):
        self.turn(cycles, -1 * velocity, description)

    def turn(self, cycles, velocity, description):
        for i in range(cycles):
            self.publish_cmd_vel(0.0, velocity, description)
            rospy.sleep(.1)

    def edge_forward(self):
        rospy.loginfo("Moving forward a short distance...")
        for i in range(15):
            self.publish_cmd_vel(settings.MIN_VELOCITY, 0.0, 'edge_forward')
            rospy.sleep(.1)

    def reset_everything(self):
        self.state = NavigationState.NONE
        self.cone_coord = None
        self.target_coord = None
        self.yerr_accumulated = 0
        self.yerr_previous = 0
    

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
                self.full_stop()
                self.move_backwards()
                self.state = NavigationState.AVOID_OBSTACLE
            elif self.obstacle_detected:
                rospy.logwarn("Obstacle in range!")
                self.full_stop()
                self.state = NavigationState.AVOID_OBSTACLE
            else:
                return self.move_towards_waypoint(goal)

        elif self.state == NavigationState.ROTATE_TO_FIND_PATH:
            self.state = NavigationState.MOVE_TOWARDS_GOAL

        elif self.state == NavigationState.AVOID_OBSTACLE:
            rospy.loginfo("Avoid obstacle mode")
            if self.obstacle_detected:
                rospy.loginfo("Obstacle still in range. Turning right")
                self.turn_to_avoid_obstacle()
                self.stop_and_sleep(.5)
            else:
                rospy.loginfo("No obstacle ahead, edging forward to clear obstacle")
                self.edge_forward()
                self.stop_and_sleep(.5)
                rospy.loginfo("Turning back towards the goal")
                self.state = NavigationState.ROTATE_INTO_POSITION

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
            self.reset_everything()
            self.server.set_succeeded()
            self.stop_and_sleep(1)
            return True

        elif math.fabs(yerr) > settings.THETA_TOLERANCE:
            # we've gone well off course... time to stop and rotate again
            rospy.logwarn("We've veered off course! rotating back into position")
            self.stop_and_sleep(1)
            self.state = NavigationState.ROTATE_INTO_POSITION
            return False
        else:
            # need to continue to move towards waypoint

            # velocity in longitudinal direction
            # (proportional control only)
            speed = settings.KP_X * xerr

            # rotational velocity is calculated using PID control

            # proportional
            rot_vel_p = settings.KP_Y * yerr

            # integral
            self.yerr_accumulated += yerr
            # don't want integral error to grow without bounds
            if math.fabs(self.yerr_accumulated) > settings.YERR_ACCUMULATED_MAX:
                if self.yerr_accumulated > 0:
                    self.yerr_accumulated = settings.YERR_ACCUMULATED_MAX
                else:
                    self.yerr_accumulated = -1 * settings.YERR_ACCUMULATED_MAX
            rot_vel_i = settings.KI_Y * self.yerr_accumulated

            # derivative
            rot_vel_d = settings.KD_Y * (yerr - self.yerr_previous)
            self.yerr_previous = yerr

            # final rotational velocity
            turnrate = rot_vel_p + rot_vel_i + rot_vel_d
#            rospy.loginfo("(yerr||p,i,d||total)= %.4f || %.4f %.4f %.4f || %.4f" % (yerr, rot_vel_p, rot_vel_i, rot_vel_d, turnrate))

            self.publish_cmd_vel(speed, turnrate, 'move_towards_waypoint')
            return False



class ConeCaptureNavigator(Navigator):
    def __init__(self, server_name):
        Navigator.__init__(self, server_name)
        self.cone_captured_pub = rospy.Publisher('cone_captured', ConeCaptured)
        self.forward_cycles = 0

    def responds_to_waypoint(self, waypoint):
        return waypoint.type == 'C'

    def capture_waypoint(self, goal):

        if self.odom is None:
            rospy.logwarn("Waiting for initial robot odometry!")
            return False

        if self.state == NavigationState.NONE:
            rospy.loginfo("Moving from %s to %s" % (NavigationState.NONE, NavigationState.ROTATE_INTO_POSITION))
            self.state = NavigationState.ROTATE_INTO_POSITION

        elif self.state == NavigationState.ROTATE_INTO_POSITION:
            self.rotate_towards_goal(goal)

        elif self.state == NavigationState.MOVE_TOWARDS_GOAL:
            # Assumption here... the intermediate waypoints are laid out in such a way
            # that by the time a cone waypoint is sent, we are close enough to the cone
            # that we should be able to see it
            self.state = NavigationState.CAPTURE_CONE
            rospy.loginfo("Moving from %s to %s" % (NavigationState.MOVE_TOWARDS_GOAL, NavigationState.CAPTURE_CONE))

        elif self.state == NavigationState.CAPTURE_CONE:

            self.flush_outdated_cone_coord_data()

            if self.collided:
                # we're done, time to return the action service
                rospy.loginfo("cone captured!")
                self.publish_cone_captured_msg(goal.waypoint)
                self.move_backwards()
                self.reset_everything()
                self.server.set_succeeded()
                return True
            elif self.cone_coord == None:
                rospy.logwarn("Attempting to capture cone but no cone_coord available!")
                self.state = NavigationState.ROTATE_TO_FIND_PATH
            elif self.forward_cycles > settings.CONE_CAPTURE_CYCLE_TIME:
                rospy.logwarn("Pausing to check for cone")
                self.stop_and_sleep(0.5)
                self.forward_cycles = 0
            else:
                self.move_towards_cone()
                self.forward_cycles += 1

        elif self.state == NavigationState.ROTATE_TO_FIND_PATH:
            could_find_cone = self.rotate_in_place_to_find_cone()
            if not could_find_cone:
                # time to abort :(
                rospy.logerr("Could not find cone! Aborting! ='(")
                self.reset_everything()
                self.server.set_aborted()
                # we're finished, so return True
                return True

        # haven't reached our goal yet...
        return False

    def move_towards_cone(self):
        """
            moves the robot forward at a constant velocity, 
            with yaw commands proportional to the error in the 
            y-direction
        """
        yerr = self.cone_coord.point.x - settings.CONE_MAX_X/2 
        rospy.loginfo("yerr=%.2f" % yerr)
        if yerr > 0.2:
            yerr = 0.2
        if yerr < -0.2:
            yerr = -0.2
        rot_vel = -1 * settings.KP_CT * yerr
        rospy.loginfo("yerr=%.2f, rot_vel=%.2f" % (yerr, rot_vel))
        self.publish_cmd_vel(settings.MIN_VELOCITY, rot_vel, 'move_towards_cone')

    def turn_left_to_find_cone(self):
        self.turn_left(settings.CONE_SEARCH_ROT_TIME, settings.CONE_SEARCH_ROT_VEL, 'turn_left_to_find_cone')

    def turn_right_to_find_cone(self):
        self.turn_right(settings.CONE_SEARCH_ROT_TIME, settings.CONE_SEARCH_ROT_VEL, 'turn_right_to_find_cone')

    def flush_outdated_cone_coord_data(self):
        if self.cone_coord is None:
            return
        
        now = rospy.Time.now().to_sec()
        latency = now - self.cone_coord.header.stamp.to_sec()
        if latency > 1.0:
            rospy.logwarn("flushing cone_coord data that is %f sec old" % latency)
            self.cone_coord = None

    def rotate_in_place_to_find_cone(self):
        self.stop_and_sleep(0.1)

        # first, try left
        self.turn_left_to_find_cone()
        if self.pause_and_check_for_cone():
            return True
        self.turn_left_to_find_cone()
        if self.pause_and_check_for_cone():
            return True
        self.turn_left_to_find_cone()
        if self.pause_and_check_for_cone():
            return True

        # come back to the original position
        self.turn_right_to_find_cone()
        self.turn_right_to_find_cone()
        self.turn_right_to_find_cone()
        if self.pause_and_check_for_cone():
            return True

        # then try right
        self.turn_right_to_find_cone()
        if self.pause_and_check_for_cone():
            return True
        self.turn_right_to_find_cone()
        if self.pause_and_check_for_cone():
            return True
        self.turn_right_to_find_cone()
        if self.pause_and_check_for_cone():
            return True

        # keep searching...
        for i in range(12):
            self.turn_right_to_find_cone()
            if self.pause_and_check_for_cone():
                return True

        return False

    def pause_and_check_for_cone(self):
        rospy.loginfo("Checking for cone...")
        self.stop_and_sleep(1)
        if self.cone_coord != None:
            rospy.loginfo("Found a cone!")
            self.stop_and_sleep(.1)
            rospy.loginfo("Switching back to %s mode" % NavigationState.CAPTURE_CONE)
            self.state = NavigationState.CAPTURE_CONE
            return True
        return False

    def publish_cone_captured_msg(self, waypoint):
        cone_captured_msg = ConeCaptured()
        cone_captured_msg.header.stamp = rospy.Time.now()
        cone_captured_msg.header.frame_id = "/map"
        cone_captured_msg.waypoint = waypoint
        self.cone_captured_pub.publish(cone_captured_msg)

