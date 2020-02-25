
import math

import rospy
import sensor_msgs

from udp_driver import UDPDriver


class MPU6050Driver(UDPDriver):

    GYROSCOPE_MEASUREMENTS_SCALE_FACTOR =  math.pi / (180. * 32.768)
    ACCELEROMETER_MEASUREMENTS_SCALE_FACTOR = 9.80665 / 4096

    def __init__(self, name='~imu_driver'):
        super(MPU6050Driver, self).__init__(name)
        self._imu_frame = rospy.get_param(rospy.resolve_name('~imu_frame', name), 'imu_link')
        self._imu_publisher = rospy.Publisher('imu', sensor_msgs.msg.Imu, queue_size=1)

    def handle_data(self, data):
        try:
            (ax, ay, az), (wx, wy, wz) = data
        except TypeError, ValueError:
            rospy.logerr('Got unexpected IMU data: %!s', data)
            return

        imu_msg = sensor_msgs.msg.Imu()
        imu_msg.header.stamp = rospy.get_rostime()
        imu_msg.header.frame_id = self._imu_frame
        imu_msg.angular_velocity.x = wx * MPU6050Driver.GYROSCOPE_MEASUREMENTS_SCALE_FACTOR
        imu_msg.angular_velocity.y = wy * MPU6050Driver.GYROSCOPE_MEASUREMENTS_SCALE_FACTOR
        imu_msg.angular_velocity.z = wz * MPU6050Driver.GYROSCOPE_MEASUREMENTS_SCALE_FACTOR
        imu_msg.linear_acceleration.x = ax * MPU6050Driver.ACCELEROMETER_MEASUREMENTS_SCALE_FACTOR
        imu_msg.linear_acceleration.y = ay * MPU6050Driver.ACCELEROMETER_MEASUREMENTS_SCALE_FACTOR
        imu_msg.linear_acceleration.z = az * MPU6050Driver.ACCELEROMETER_MEASUREMENTS_SCALE_FACTOR

        self._imu_publisher.publish(imu_msg)
