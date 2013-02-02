#!/usr/bin/env python

#
# for testing out dynamic reconfigure
#

import roslib; roslib.load_manifest('robomagellan')
import rospy

from dynamic_reconfigure.server import Server
from robomagellan.msg import ParametersConfig


def callback(config, level):
    rospy.loginfo("""Reconfiugre Request: {int_param}, {double_param},\ 
          {str_param}, {bool_param}, {size}""".format(**config))
    return config


if __name__ == '__main__':
    rospy.init_node('parameter_test')

    srv = Server(ParametersConfig, callback)
    rospy.spin()
            
