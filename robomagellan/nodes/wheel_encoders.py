
# NOT A FUNCTIONING NODE... JUST NOTES FOR NOW   
 
# from rooma_sensor_handler.py

    self._last_encoder_counts = None

  def _normalize_encoder_count(self, count_delta, maximal_count):
    if count_delta >= maximal_count / 2:
      return count_delta - maximal_count + 1
    elif count_delta <= -maximal_count / 2:
      return count_delta + maximal_count + 1
    return count_delta

    # The distance and angle calculation sent by the robot seems to
    # be really bad. Re-calculate the values using the raw enconder
    # counts.
    if self._last_encoder_counts:
      count_delta_left = self._normalize_encoder_count(
          self.encoder_counts_left - self._last_encoder_counts[0], 0xffff)
      count_delta_right = self._normalize_encoder_count(
          self.encoder_counts_right - self._last_encoder_counts[1], 0xffff)
      distance_left = count_delta_left * self.ROOMBA_PULSES_TO_M
      distance_right = count_delta_right * self.ROOMBA_PULSES_TO_M
      self.distance = (distance_left + distance_right) / 2.0
      self.angle = (distance_right - distance_left) / robot_types.ROBOT_TYPES['roomba'].wheel_separation
    else:
      self.disance = 0
      self.angle = 0
    self._last_encoder_counts = (self.encoder_counts_left, self.encoder_counts_right)

 

# from turtlebot_node.py
    def compute_odom(self, sensor_state, last_time, odom):
        """
        Compute current odometry.  Updates odom instance and returns tf
        transform. compute_odom() does not set frame ids or covariances in
        Odometry instance.  It will only set stamp, pose, and twist.

        @param sensor_state: Current sensor reading
        @type  sensor_state: TurtlebotSensorState
        @param last_time: time of last sensor reading
        @type  last_time: rospy.Time
        @param odom: Odometry instance to update.
        @type  odom: nav_msgs.msg.Odometry

        @return: transform
        @rtype: ( (float, float, float), (float, float, float, float) )
        """
        # based on otl_roomba by OTL <t.ogura@gmail.com>

        current_time = sensor_state.header.stamp
        dt = (current_time - last_time).to_sec()

        # On startup, Create can report junk readings
        if abs(sensor_state.distance) > 1.0 or abs(sensor_state.angle) > 1.0:
            raise Exception("Distance, angle displacement too big, invalid readings from robot. Distance: %.2f, Angle: %.2f" % (sensor_state.distance, sensor_state.angle))

        # this is really delta_distance, delta_angle
        d  = sensor_state.distance * self.odom_linear_scale_correction #correction factor from calibration
        angle = sensor_state.angle * self.odom_angular_scale_correction #correction factor from calibration

        x = cos(angle) * d
        y = -sin(angle) * d

        last_angle = self._pos2d.theta
        self._pos2d.x += cos(last_angle)*x - sin(last_angle)*y
        self._pos2d.y += sin(last_angle)*x + cos(last_angle)*y
        self._pos2d.theta += angle

        # Turtlebot quaternion from yaw. simplified version of tf.transformations.quaternion_about_axis
        odom_quat = (0., 0., sin(self._pos2d.theta/2.), cos(self._pos2d.theta/2.))

        # construct the transform
        transform = (self._pos2d.x, self._pos2d.y, 0.), odom_quat

        # update the odometry state
        odom.header.stamp = current_time
        odom.pose.pose   = Pose(Point(self._pos2d.x, self._pos2d.y, 0.), Quaternion(*odom_quat))
        odom.twist.twist = Twist(Vector3(d/dt, 0, 0), Vector3(0, 0, angle/dt))
        if sensor_state.requested_right_velocity == 0 and \
               sensor_state.requested_left_velocity == 0 and \
               sensor_state.distance == 0:
            odom.pose.covariance = ODOM_POSE_COVARIANCE2
            odom.twist.covariance = ODOM_TWIST_COVARIANCE2
        else:
            odom.pose.covariance = ODOM_POSE_COVARIANCE
            odom.twist.covariance = ODOM_TWIST_COVARIANCE

        # return the transform
        return transform

    def publish_odometry_transform(self, odometry):
        self.transform_broadcaster.sendTransform(
            (odometry.pose.pose.position.x, odometry.pose.pose.position.y, odometry.pose.pose.position.z),
            (odometry.pose.pose.orientation.x, odometry.pose.pose.orientation.y, odometry.pose.pose.orientation.z,
             odometry.pose.pose.orientation.w),
             odometry.header.s
