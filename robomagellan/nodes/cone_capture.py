#!/usr/bin/env python

"""

cone_capture
  based on the location of a cone published by the cone_tracker node,
  this node will send commands to the robot base on the /cmd_vel topic,
  as it attempts to reach and physically contact the cone.

listens to:
 /cone_coord
 /collision

publishes to:
 /cmd_vel

TODO:
  add actionlib service call, similar to move_base

"""

import roslib; roslib.load_manifest('robomagellan')
import rospy

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped
from robomagellan.msg import BoolStamped

import tf

import settings

class ConeCapturer():
    def __init__(self, publisher):
        self.cone_coord = None
        self.collided = False
        self.publisher = publisher
        self.transformListener = tf.TransformListener()
        return

    def setup_cone_coord_callback(self):
        def cone_coord_callback(data):
            self.cone_coord = data
        return cone_coord_callback
                    
    def setup_collision_callback(self):
        def collision_callback(data):
            self.collided = data.value
        return collision_callback

    def capture_cone(self):
        """
            moves the robot forward at a constant velocity, 
            with yaw commands proportional to the error in the 
            y-direction
        """
        if self.collided:
            # we're done, time to return the action service
            rospy.loginfo("cone captured!")
            cmd_vel = Twist()
            self.publisher.publish(cmd_vel)
            return

        # TODO check time difference here as well
        if self.cone_coord == None:
#            rospy.logwarn("Attempting to capture cone but no cone_coord available!")
            return 
            
        self.transformListener.transformPoint("base_link", self.cone_coord)
        cmd_vel = Twist()
        cmd_vel.linear.x = settings.SPEED_TO_CAPTURE
        cmd_vel.angular.z = self.cone_coord.point.y
        self.publisher.publish(cmd_vel)


if __name__ == '__main__':
    rospy.init_node('cone_capture')
    rospy.sleep(3)  # let rxconsole boot up
    rospy.loginfo("Initializing cone_capture node")

    publisher = rospy.Publisher('cmd_vel', Twist)
    capturer = ConeCapturer(publisher)
    rospy.Subscriber('cone_coord', PointStamped, capturer.setup_cone_coord_callback())
    rospy.Subscriber('collision', BoolStamped, capturer.setup_collision_callback())
            
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        rate.sleep()
        capturer.capture_cone()
