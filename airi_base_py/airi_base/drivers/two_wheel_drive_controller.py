
import rospy

import std_msgs

from udp_driver import UDPDriver


class TwoWheelDriveController(UDPDriver):

    def __init__(self, name, *args, **kwargs):
        super(TwoWheelDriveController, self).__init__(name, *args, **kwargs)
        self._left_encoder_pulses_publisher = rospy.Publisher(
            'encoders/left/pulses', std_msgs.msg.Int32, queue_size=1
        )
        self._right_encoder_pulses_publisher = rospy.Publisher(
            'encoders/right/pulses', std_msgs.msg.Int32, queue_size=1
        )

    def drive(self, left_wheel_pps, right_wheel_pps):
        self.send_data([left_wheel_pps, right_wheel_pps])

    def handle_data(self, data):
        try:
            left_encoder_pulses, right_encoder_pulses = data
        except TypeError, ValueError:
            rospy.logerr("%s got unexpected wheel drive data: '%!s'", self.name, data)
            return
        self.on_encoder_data(left_encoder_pulses, right_encoder_pulses)

        self._left_encoder_pulses_publisher.publish(
            std_msgs.msg.Int32(data=left_encoder_pulses)
        )

        self._right_encoder_pulses_publisher.publish(
            std_msgs.msg.Int32(data=right_encoder_pulses)
        )

    def on_encoder_data(self, left_encoder_pulses, right_encoder_pulses):
        pass
