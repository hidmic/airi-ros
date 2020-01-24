
import rospy

import geometry_msgs
import nav_msgs
import tf2_ros

import tf_conversions

from two_wheel_driver_controller import TwoWheelDriveController


class DifferentialDriveController(TwoWheelDriveController):

    def __init__(self, name='~drive_controller'):
        super(DifferentialDriveController, self).__init__(name)
        self._wheel_base = rospy.get_param(rospy.resolve_name('~wheel_base', name))
        self._wheel_diameter = rospy.get_param(rospy.resolve_name('~wheel_diameter', name))
        self._wheel_encoder_resolution = rospy.get_param(
            rospy.resolve_name('~wheel_encoder_resolution', name)
        )
        self._wheel_base = rospy.get_param(rospy.resolve_name('~odom_frame', name), 'odom')
        self._wheel_base = rospy.get_param(rospy.resolve_name('~base_frame', name), 'base_footprint')
        self._publish_tf = rospy.get_param(rospy.resolve_name('~publish_tf', name), True)

        if self._publish_tf:
            self._tf_broadcaster = tf2_ros.TransformBroadcaster()

        self._last_update_time = None
        self._last_update_pose = None
        self._last_encoder_data = None
        self._odom_publisher = rospy.Publisher(
            'odom', nav_msgs.msg.Odometry, queue_size=1
        )

        self._cmd_subscriber = rospy.Subscriber(
            'cmd_vel', geometry_msgs.msg.Twist, self.on_velocity_command
        )

    def on_velocity_command(self, twist_msg):
        if not self.is_connected():
            rospy.loginfo('%s got a velocity command but it is not running', self.name)
            return
        # Compute pulses per second per wheel to sustain
        right_wheel_encoder_pps = self._wheel_encoder_resolution * (
            2 * twist_msg.linear.x + twist_msg.angular.z * self._wheel_base
        ) / (self._wheel_diameter * math.pi)
        left_wheel_encoder_pps = self._wheel_encoder_resolution * (
            2 * twist_msg.linear.x - twist_msg.angular.z * self._wheel_base
        ) / (self._wheel_diameter * math.pi)

        self.drive(left_wheel_encoder_pps, left_wheel_encoder_pps)

    def on_encoder_data(self, left_wheel_encoder_pulses, right_wheel_encoder_pulses):
        now = rospy.get_rostime()

        if self._last_update_time is not None:
            dt = (now - self._last_update_time).to_sec()
            last_left_wheel_encoder_pulses, last_right_wheel_encoder_pulses = self._last_encoder_data
            left_wheel_encoder_delta = left_wheel_encoder_pulses - last_left_wheel_encoder_pulses
            right_wheel_encoder_delta = right_wheel_encoder_pulses - last_right_wheel_encoder_pulses

            vk = math.pi * self._wheel_diameter * (
                left_wheel_encoder_delta + right_wheel_encoder_delta
            ) / (2 * self._wheel_encoder_resolution)

            wk = math.pi * self._wheel_diameter * (
                left_wheel_encoder_delta - right_wheel_encoder_delta
            ) / (self._wheel_base * self._wheel_encoder_resolution)

            xk_1, yk_1, thetak_1 = self._last_update_pose

            xk = xk_1 + vk * dt * math.cos(thetak_1)
            yk = yk_1 + vk * dt * math.sin(thetak_1)
            thetak = thetak_1 + wk * dt
        else:
            xk = yk = thetak = vk = wk = 0.0

        self._last_update_time = now
        self._last_odom_pose = xk, yk, thetak
        self._last_encoder_data = left_wheel_encoder_pulses, right_wheel_encoder_pulses

        qk = tf_conversions.transformations.quaternion_from_euler(0, 0, thetak)

        odom = nav_msgs.msg.Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id = self._base_frame
        odom.pose.pose.position.x = xk
        odom.pose.pose.position.y = yk
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = qk[0]
        odom.pose.pose.orientation.y = qk[1]
        odom.pose.pose.orientation.z = qk[2]
        odom.pose.pose.orientation.w = qk[3]
        odom.twist.twist.linear.x = vk
        odom.twist.twist.linear.w = wk
        self._odom_publisher.publish(odom)

        if self._publish_tf:
            tf = geometry_msgs.msg.TransformStamped()
            tf.header.stamp = now
            tf.header.frame_id = self._odom_frame
            tf.child_frame_id = self._base_frame
            tf.transform.translation.x = xk
            tf.transform.translation.y = yk
            tf.transform.translation.z = 0.0
            tf.transform.rotation.x = qk[0]
            tf.transform.rotation.y = qk[1]
            tf.transform.rotation.z = qk[2]
            tf.transform.rotation.w = qk[3]
            self._tf_broadcaster.sendTransform(tf)
