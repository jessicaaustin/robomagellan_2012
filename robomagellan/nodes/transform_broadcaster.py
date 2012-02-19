#!/usr/bin/env python

"""

transform_broadcaster.py

broadcasts static transforms for the robot

"""

import roslib; roslib.load_manifest('robomagellan')
import rospy

import tf

from math import pi

def broadcast_camera_static_frames(br):

    # from robot_base to camera_lens
    # TODO update with correct offsets once camera is mounted on robot
    br = tf.TransformBroadcaster()
    br.sendTransform((0.0, 0.0, 0.0),
                 (0.0, 0.0, 0.0, 1.0),
                 rospy.Time.now(),
                 "camera_lens",
                 "robot_base")

    # from camera_lens to camera_lens_optical
    # rotate so that the z axis is coming out of the camera lens
    br = tf.TransformBroadcaster()
    br.sendTransform((0, 0, 0),
                 tf.transformations.quaternion_from_euler(-pi/2.0, 0, 0),
                 rospy.Time.now(),
                 "camera_lens_optical",
                 "camera_lens")


if __name__ == '__main__':
    rospy.init_node('transforms')
    
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        broadcast_camera_static_frames(br)
        rate.sleep()

