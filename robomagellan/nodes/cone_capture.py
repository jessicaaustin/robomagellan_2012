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

import tf
import actionlib

from robomagellan.msg import CaptureConeAction

import settings

class ConeCapturer():
    def __init__(self, publisher):
        self.cone_coord = None
        self.collided = False
        self.publisher = publisher
        self.transformListener = tf.TransformListener()
        self.server = actionlib.SimpleActionServer('capture_cone', CaptureConeAction, self.execute, False)
        self.server.start()
        return

    def setup_cone_coord_callback(self):
        def cone_coord_callback(data):
            self.cone_coord = data
        return cone_coord_callback
                    
    def setup_collision_callback(self):
        def collision_callback(data):
            self.collided = data.value
        return collision_callback

    def execute(self, goal):
        rospy.loginfo("Received goal: %s" % goal)

        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            if self.collided:
                # we're done, time to return the action service
                # TODO back up a little bit before returning, so that we clear the obstacle
                rospy.loginfo("cone captured!")
                cmd_vel = Twist()
                self.publisher.publish(cmd_vel)
                self.server.set_succeeded()
                return
            # TODO check time difference here as well, we don't want to use the
            #      cone_coord data if it is very old
            elif self.cone_coord == None:
                rospy.logwarn("Attempting to capture cone but no cone_coord available!")
                # TODO abort if we can't find the cone after some amount of time
            else:
                self.move_towards_cone()

            rate.sleep()

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


if __name__ == '__main__':
    rospy.init_node('cone_capture')
    rospy.sleep(3)  # let rxconsole boot up
    rospy.loginfo("Initializing cone_capture node")

    publisher = rospy.Publisher('cmd_vel', Twist)
    capturer = ConeCapturer(publisher)
    rospy.Subscriber('cone_coord', PointStamped, capturer.setup_cone_coord_callback())
    rospy.Subscriber('collision', BoolStamped, capturer.setup_collision_callback())
            
    rospy.spin()
