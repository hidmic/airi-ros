import rospy

import sensor_msgs

from udp_driver import UDPDriver


class HCSR04SonarRingDriver(UDPDriver):

    RANGE_UNITS_TO_M_SCALE_FACTOR = 1.0 / 58000

    def __init__(self, name='~sonar_ring_driver'):
        super(UDPDriver, self).__init__(name)
        self._sonar_topics = rospy.get_param(rospy.resolve_name('~sonar_topics', name), ['sonar'])
        self._sonar_frames = rospy.get_param(rospy.resolve_name('~sonar_frames', name), ['sonar_link'])
        if len(self._sonar_topics) != len(self._sonar_frames):
            raise RuntimeError(
                "Sonar topics' count ({}) and frames' count ({}) for {} do not match".format(
                    len(self._sonar_topics), len(self._sonar_frames), self.name
                )
            )
        self._range_publishers = [
            rospy.Publisher(topic, sensor_msgs.msg.Range, queue_size=1) for topic in self._sonar_topics
        ]

    def incoming_data(self, data):
        try:
            measured_ranges = [r * HCSR04SonarRingDriver.RANGE_UNITS_TO_M_SCALE_FACTOR for r in data]
        except (TypeError, ValueError):
            rospy.logerr('%s got unexpected range data: %!s', self.name, data)
            return
        if len(measured_ranges) != len(self._range_publishers):
            rospy.logwarn(
                '%s expected %d range readings, got %d', self.name,
                len(self._range_publishers), len(measured_ranges)
            )
        for publisher, frame, measured_range in zip(
                self._range_publishers, self._sonar_frames, measured_ranges
        ):
            sonar_msg = sensor_msgs.msg.Range()
            sonar_msg.header.stamp = rospy.get_rostime()
            sonar_msg.header.frame_id = frame
            sonar_msg.radiation_type = sensor_msgs.msg.Range.ULTRASOUND
            sonar_msg.fov = 30.0 * math.pi / 180.0
            sonar_msg.min_range = 0.02
            sonar_msg.max_range = 2.00
            sonar_msg.range = measured_range
            publisher.publish(sonar_msg)
