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

import tf
import actionlib

from robomagellan.msg import CaptureWaypointAction
from robomagellan.msg import CaptureWaypointFeedback

import settings

class Navigator():
    def __init__(self, publisher):
        self.publisher = publisher
        self.transformListener = tf.TransformListener()
        self.cone_coord = None
        self.collided = False
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
            # TODO check here if the goal has been pre-empted
            finished = self.capture_waypoint(goal)
            self.publish_feedback()
            rate.sleep()

    def setup_cone_coord_callback(self):
        def cone_coord_callback(data):
            # only save this data if we are moving towards a cone 
            # we do this to avoid outdated cone data
            if self.state == NavigationState.CAPTURE_CONE:
                self.cone_coord = data
            else:
                self.cone_coord = None
        return cone_coord_callback
                    
    def setup_collision_callback(self):
        def collision_callback(data):
            self.collided = data.value
        return collision_callback

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
       


class ConeCaptureNavigator(Navigator):
    def __init__(self, publisher):
        Navigator.__init__(self, publisher)

    def responds_to_waypoint(self, waypoint):
        return waypoint.type == 'C'

    def capture_waypoint(self, goal):

        if self.state == NavigationState.NONE:
            self.state = NavigationState.ROTATE_INTO_POSITION

        elif self.state == NavigationState.ROTATE_INTO_POSITION:
            self.rotate_towards_goal()
            self.state = NavigationState.MOVE_TOWARDS_GOAL

        elif self.state == NavigationState.MOVE_TOWARDS_GOAL:
            self.state = NavigationState.CAPTURE_CONE

        elif self.state == NavigationState.CAPTURE_CONE:
            if self.collided:
                # we're done, time to return the action service
                rospy.loginfo("cone captured!")
                self.move_backwards_to_clear_obstacle()
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
            # TODO abort if we can't find the cone after some amount of time

        # haven't reached our goal yet...
        return False

    def rotate_towards_goal(self):
        return

    def move_towards_cone(self):
        """
            moves the robot forward at a constant velocity, 
            with yaw commands proportional to the error in the 
            y-direction
        """

        self.transformListener.transformPoint("base_link", self.cone_coord)
        cmd_vel = Twist()
        cmd_vel.linear.x = settings.SPEED_TO_CAPTURE
        # TODO proportional control will probably yield terrible results here...
        #      switch to PID control instead
        cmd_vel.angular.z = self.cone_coord.point.y
        if cmd_vel.angular.z > 0.2:
            cmd_vel.angular.z = 0.2
        if cmd_vel.angular.z < -0.2:
            cmd_vel.angular.z = -0.2
        self.publisher.publish(cmd_vel)

    def move_backwards_to_clear_obstacle(self):
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
        return


if __name__ == '__main__':
    rospy.init_node('cone_capture')
    rospy.sleep(3)  # let rxconsole boot up
    rospy.loginfo("Initializing cone_capture node")

    publisher = rospy.Publisher('cmd_vel', Twist)
    capturer = ConeCaptureNavigator(publisher)
    rospy.Subscriber('cone_coord', PointStamped, capturer.setup_cone_coord_callback())
    rospy.Subscriber('collision', BoolStamped, capturer.setup_collision_callback())
            
    rospy.spin()
