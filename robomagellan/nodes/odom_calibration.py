#!/usr/bin/env python

"""

odom_calibration
  sends a set of motion commands, for use in calibrating odometry

publishes to:
 /cmd_vel

"""

import roslib; roslib.load_manifest('robomagellan')
import rospy

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

from nav_msgs.msg import Path

import settings
import tf

import math

class RoverCommander():
    def __init__(self):
        self.odom = None
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist)
        self.cmd_vel_path_pub = rospy.Publisher('navigator/local_plan', Path)
 
    def command_full_stop(self):
        cmd_vel = Twist()
        self.cmd_vel_pub.publish(cmd_vel)
        self.publish_cmd_vel_path(cmd_vel)

    def command_move_forward(self, distance):
        velocity = 0.4  # m/s
        timesteps = (int) (distance / velocity / 0.1)
        for i in range(timesteps):
            self.publish_cmd_vel(velocity, 0.0, 'move_forward')
            rospy.sleep(.1)
        self.command_full_stop()

    def command_rotate_left(self):
        for i in range(settings.CONE_SEARCH_ROT_TIME):
            self.publish_cmd_vel(0.0, settings.CONE_SEARCH_ROT_VEL, 'turn_left')
            rospy.sleep(.1)
        self.command_full_stop()

    def command_rotate_right(self):
        for i in range(settings.CONE_SEARCH_ROT_TIME):
            self.publish_cmd_vel(0.0, -1 * settings.CONE_SEARCH_ROT_VEL, 'turn_right')
            rospy.sleep(.1)
        self.command_full_stop()


    # helpers
     
    def setup_odom_callback(self):
        def odom_callback(data):
            self.odom = data
        return odom_callback

    def publish_cmd_vel(self, x, z, source):
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

if __name__ == '__main__':
    rospy.init_node('odom_calibration')
    rover_commander = RoverCommander()

    rospy.Subscriber('odom', Odometry, rover_commander.setup_odom_callback())
    while rover_commander.odom is None:
        rospy.logwarn("Waiting for odometry!")
        rospy.sleep(.1)

    stepDelay = 1.5

    rospy.logwarn("stopping")
    rover_commander.command_full_stop()
    rospy.sleep(stepDelay)

    rospy.logwarn("moving forward 4.0 meter")
    rover_commander.command_move_forward(1.0)
    rospy.sleep(stepDelay)

    rospy.logwarn("rotating left")
    rover_commander.command_rotate_left()
    rospy.sleep(stepDelay)
    rover_commander.command_rotate_left()
    rospy.sleep(stepDelay)
    rover_commander.command_rotate_left()
    rospy.sleep(stepDelay)

    rospy.logwarn("rotating right")
    rover_commander.command_rotate_right()
    rospy.sleep(stepDelay)
    rover_commander.command_rotate_right()
    rospy.sleep(stepDelay)
    rover_commander.command_rotate_right()
    rospy.sleep(stepDelay)
    rover_commander.command_rotate_right()
    rospy.sleep(stepDelay)
    rover_commander.command_rotate_right()
    rospy.sleep(stepDelay)
    rover_commander.command_rotate_right()
    rospy.sleep(stepDelay)

    rospy.logwarn("rotating left")
    rover_commander.command_rotate_left()
    rospy.sleep(stepDelay)
    rover_commander.command_rotate_left()
    rospy.sleep(stepDelay)
    rover_commander.command_rotate_left()
    rospy.sleep(stepDelay)
    rover_commander.command_full_stop()

    rospy.logwarn("Done.")

