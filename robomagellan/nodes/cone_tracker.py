#!/usr/bin/env python

"""

cone_tracker
- detects and tracks orange traffic cones as specified in the robomagellan rules:
  "The destination and bonus waypoints will be designated with 
   latitude/longitude coordinates and marked by 18", orange, 
   plastic traffic cones." 
- publishes to a topic (cone_coord) the coords of the cone
- also publishes a marker on cone_marker topic
- does NOT attempt to control the robot based on cone location--see cone_capture.py for that

How it works:
- uses opencv framework
- detects areas of orange in the image
- finds areas of orange and uses shape and size to determine likely cone candidates
- chooses the largest blob as the cone candidate
- uses position in the plane of the camera frame to determine x,y position, and the size of the cone blob to determine z (depth) 

Assumptions:
- there are no other orange objects in the vicinity (so the most significant orange object in the FOV must be the cone)

"""

import roslib; roslib.load_manifest('robomagellan')
import rospy

from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Point
from rospy.exceptions import ROSInitException
from visualization_msgs.msg import Marker
from robomagellan.msg import AsciiArt
from sensor_msgs.msg import Image
import tf

import cv
import operator

from cv_bridge import CvBridge

import settings
from waypoint_reader import WaypointFileReader

import os
import sys
import math


class ConeTracker():
    def __init__(self, camera_index, min_thresh, max_thresh, smoothness):
        # initialize camera feed
        self.capture = cv.CaptureFromCAM(camera_index)
        if not self.capture:
            err = "Could not init camera feed! camera_index=%s" % camera_index
            rospy.logerr(err)
            raise ROSInitException(err)

        # store the image capture params
        self.smoothness = smoothness
        self.min_thresh = min_thresh
        self.max_thresh = max_thresh

        # initialize position array
        self.positions_x, self.positions_y = [0]*smoothness, [0]*smoothness

        # setup cv bridge, for converting from opencv images to ROS images
        self.bridge = CvBridge()
        self.pub_image = rospy.Publisher("camera_image", Image)
        self.pub_image_threshed = rospy.Publisher("camera_image_threshed", Image)

        rospy.loginfo("Successfully initialized ConeTracker")

    def pixel_to_meter(self, dist):
        return dist / settings.PIXELS_TO_CM / 100

    def find_blob(self):
        """
        captures a frame from the camera
        attempts to find the blob
        publishes the current blob location, if the blob was found
        """
        image = cv.QueryFrame(self.capture)
        if not image:
            rospy.logerr("Could not capture image")
            return None

        # smooth the image
        image_smoothed = cv.CloneImage(image)
        cv.Smooth(image, image_smoothed, cv.CV_GAUSSIAN, 15)
        # threshold the smoothed image
        image_threshed = self.thresholded_image(image_smoothed)
        
        # blobify
        cv.Dilate(image_threshed, image_threshed, None, 18)
        cv.Erode(image_threshed, image_threshed, None, 10)

        blobContour = None

        # extract the edges from our binary image
        current_contour = cv.FindContours(cv.CloneImage(image_threshed), cv.CreateMemStorage(), cv.CV_RETR_CCOMP, cv.CV_CHAIN_APPROX_SIMPLE)
        # if there is a matching object in the frame
        if len(current_contour) != 0:

            # find the largest blob
            largest_contour = current_contour
            while True:
                current_contour = current_contour.h_next()
                if (not current_contour):
                    break
                if (cv.ContourArea(current_contour) > cv.ContourArea(largest_contour)):
                    largest_contour = current_contour

            # if we got a good enough blob
            if cv.ContourArea(largest_contour)>2.0:
                blobContour = largest_contour
                # find the center of the blob
                moments = cv.Moments(largest_contour, 1)
                self.positions_x.append(cv.GetSpatialMoment(moments, 1, 0)/cv.GetSpatialMoment(moments, 0, 0))
                self.positions_y.append(cv.GetSpatialMoment(moments, 0, 1)/cv.GetSpatialMoment(moments, 0, 0))
                # discard all but the last N positions
                self.positions_x, self.positions_y = self.positions_x[-self.smoothness:], self.positions_y[-self.smoothness:]

        # the average location of the identified blob
        pos_x = int(sum(self.positions_x)/len(self.positions_x))
        pos_y = int(sum(self.positions_y)/len(self.positions_y))
        object_position = (pos_x,pos_y)

        # If we found a blob, overlay the location on the original image
        if blobContour:
            desiredPosition = cv.GetSize(image)
            desiredPosition = tuple(map(operator.mul, desiredPosition, (0.5, 0.5)))
            desiredPosition = tuple(map(int, desiredPosition))

            # draw a line to the desiredPosition
            object_indicator = cv.CreateImage(cv.GetSize(image), image.depth, image.nChannels)
            cv.SetZero(object_indicator)
            cv.Circle(object_indicator, desiredPosition, 4, (255,0,0), 2)
            cv.Line(object_indicator, object_position, desiredPosition, (255,0,0), 3) 
            cv.Circle(object_indicator, object_position, 8, (0,255,0), 2)
            cv.Add(image, object_indicator, image)

        # publish the image
        self.pub_image.publish(self.bridge.cv_to_imgmsg(image))
        self.pub_image_threshed.publish(self.bridge.cv_to_imgmsg(image_threshed))

        if blobContour:
            z = 1.0  # we assume the ball is 1 meter away (no depth perception yet!)
            return Point(self.pixel_to_meter(pos_x), self.pixel_to_meter(pos_y), z)
        else:
            return None


    def thresholded_image(self, image):
        """
        convert the given image to a binary image where all values are 
        zero other than areas with hue we're looking for
        """
        # convert image to hsv
        image_hsv = cv.CreateImage(cv.GetSize(image), image.depth, 3)
        cv.CvtColor(image, image_hsv, cv.CV_BGR2HSV)
        # threshold the image
        image_threshed = cv.CreateImage(cv.GetSize(image), image.depth, 1)
        cv.InRangeS(image_hsv, self.min_thresh, self.max_thresh, image_threshed)
        return image_threshed


# Simulates a camera node, using the input waypoint file to determine when
# a cone might been tracked in the camera frame.
#
# how it works:
#  get waypoints from waypoints file, which are in /map frame
#  iterate over all the points
#    use tf.TransformerROS.transformPoint to convert each point into the /camera_lens_optical frame
#    if the distance is small enough, return the point
# 
class ConeTrackerSim():
    def __init__(self, waypoints_file):
        self.transformListener = tf.TransformListener()

        waypoint_file_reader = WaypointFileReader()
        waypoints = waypoint_file_reader.read_file(waypoints_file)
        self.waypoints = []
        for w in waypoints:
            if w.type == 'C':
                point = PointStamped()
                point.header.frame_id = "map"
                point.point.x = w.coordinate.x
                point.point.y = w.coordinate.y
                point.point.z = 0.22  # cones are 18" tall, halfway up is 9" = .22m
                self.waypoints.append(point)

        if len(self.waypoints) == 0:
            rospy.logerr("ConeTrackerSim initialized with zero waypoints! Nothing will be published...")
            return

        # wait for the required frames to start being published
        try:
            self.transformListener.waitForTransform("camera_lens_optical", "map", rospy.Time(), rospy.Duration(10))
        except tf.Exception:
            rospy.logerr("Timeout waiting for required frames! Nothing will be published...")
            return

        rospy.loginfo("ConeTrackerSim initialized with waypoints: %s" % self.waypoints)


    def find_blob(self):
        for waypoint in self.waypoints:
            waypoint.header.stamp = self.transformListener.getLatestCommonTime("camera_lens_optical", "map")
            point_in_camera_frame = self.transformListener.transformPoint("camera_lens_optical", waypoint)
            x, y, z = point_in_camera_frame.point.x, point_in_camera_frame.point.y, point_in_camera_frame.point.z
            if (z < 5 and math.fabs(y) < .45*z and math.fabs(x) < .35*z):
                p = Point()
                p.z = 1.0  # we assume the ball is 1 meter away (no depth perception yet!)
                p.y = x
                p.x = 0.1 - y/(4.0*z)
                rospy.loginfo("(x,y,z)=(%.2f,%.2f,%.2f) | (%.2f,%.2f,%.2f)" % (x,y,z,p.x, p.y, p.z))
                return p
        return None



if __name__ == '__main__':
    rospy.init_node('cone_tracker')
    rospy.sleep(3)  # let rxconsole boot up
    rospy.loginfo("Initializing cone_tracker node")

    if len(sys.argv) >= 2 and os.path.exists(sys.argv[1]) and os.path.isfile(sys.argv[1]):
        # running in simulation mode
        rospy.loginfo("Running cone_tracker in simulation mode")
        waypoints_file = sys.argv[1]
        cone_tracker = ConeTrackerSim(waypoints_file)
    else:
        # running in normal mode
        rospy.loginfo("Running cone_tracker in production mode")
        cone_tracker = ConeTracker(settings.MY_CAMERA, settings.MIN_THRESH, settings.MAX_THRESH, settings.SMOOTHNESS)

    pub_cone_coord = rospy.Publisher('cone_coord', PointStamped)
    pub_cone_marker = rospy.Publisher('cone_marker', Marker)
    pub_cone_coord_ascii = rospy.Publisher('cone_coord_ascii', AsciiArt)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        cone_coord = cone_tracker.find_blob()
        if cone_coord:
            cone_coord_stamped = PointStamped()
            cone_coord_stamped.header.frame_id = "camera_lens_optical"
            cone_coord_stamped.header.stamp = rospy.Time.now()
            cone_coord_stamped.point = cone_coord
            pub_cone_coord.publish(cone_coord_stamped)
            
            cone_marker = Marker()
            cone_marker.type = 2  # Sphere
            cone_marker.scale.x = 0.08
            cone_marker.scale.y = 0.08
            cone_marker.scale.z = 0.08
            cone_marker.color.r = 0.0
            cone_marker.color.g = 1.0
            cone_marker.color.b = 0.0
            cone_marker.color.a = 1.0
            cone_marker.header.frame_id = "camera_lens_optical"
            cone_marker.header.stamp = rospy.Time.now()
            cone_marker.pose.position = cone_coord
            pub_cone_marker.publish(cone_marker)

            cone_coord_ascii = AsciiArt()
            cone_coord_ascii.header.frame_id= "camera_lens_optical"
            cone_coord_ascii.header.stamp = rospy.Time.now()
            max_x, max_y = (int)(settings.CONE_MAX_X * 100), (int)((settings.CONE_MAX_Y * 100)/2)
            ascii_x = (int)(cone_coord.x * 100)
            ascii_y = (int)((cone_coord.y * 100)/2)
            art = "\n"
            for j in range(0, max_y):
                art += "|"
                for i in range(0, max_x):
                    if i == ascii_x and j == ascii_y:
                        art += "O"
                    elif i == ((int)(max_x/2)) and j == ((int)(max_y/2)):
                        art += "+"
                    else:
                        art += " "
                art += "|\n"
            cone_coord_ascii.art = art
            pub_cone_coord_ascii.publish(cone_coord_ascii)

        rate.sleep()
            
