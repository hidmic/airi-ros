import rospy

import sensor_msgs

from udp_driver import UDPDriver


INF = float('inf')

class KY033RangerRingDriver(UDPDriver):

    def __init__(self, name='~ranger_ring_driver'):
        super(KY033RangerRingDriver, self).__init__(name)
        self._ranger_topics = rospy.get_param(rospy.resolve_name('~ranger_topics', name), ['ranger'])
        self._ranger_frames = rospy.get_param(rospy.resolve_name('~ranger_frames', name), ['ranger_link'])
        if len(self._ranger_topics) != len(self._ranger_frames):
            raise RuntimeError(
                "Ranger topics' count ({}) and frames' count ({}) for {} do not match".format(
                    len(self._sonar_topics), len(self._sonar_frames), self.name
                )
            )
        self._range_publishers = [
            rospy.Publisher(topic, sensor_msgs.msg.Range, queue_size=1) for topic in self._ranger_topics
        ]

    def handle_data(self, data):
        try:
            detections = [bool(value) for value in data]
        except (TypeError, ValueError):
            rospy.logerr('%s got unexpected range data: %!s', self.name, data)
            return

        for publisher, frame, detection in zip(
            self._range_publishers, self._ranger_frames, detections
        ):
            range_msg = sensor_msgs.msg.Range()
            range_msg.header.stamp = rospy.get_rostime()
            range_msg.header.frame_id = frame
            range_msg.radiation_type = sensor_msgs.msg.Range.INFRARED
            range_msg.fov = 0.
            range_msg.min_range = -INF
            range_msg.max_range = INF
            range_msg.range = INF if detection else -INF
            publisher.publish(range_msg)
