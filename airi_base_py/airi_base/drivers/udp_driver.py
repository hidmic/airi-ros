
import os
import msgpack
import rospy
import select
import socket
import threading


class UDPDriver(threading.Thread):

    def __init__(self, name='', buffer_size=512, liveliness_duration=0.2):
        super(UDPDriver, self).__init__(name=rospy.resolve_name(name))
        self._host = rospy.get_param(rospy.resolve_name('~host', name))
        self._port = rospy.get_param(rospy.resolve_name('~port', name))
        self._inbound_traffic = self._outbound_traffic = False
        self._liveliness_duration = liveliness_duration
        self._handlers = {'ctrl': [self._handle_control]}
        self._control_handlers = {'shutdown': [self._handle_shutdown]}
        self._buffer = bytearray(buffer_size)
        self._is_shutdown = True

    def run(self):
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        shutdown_listener_fd, self._shutdown_notifier_fd = os.pipe()
        self._send_packet({'ctrl': 'connect'})
        liveliness_check_timer = rospy.Timer(
            self._liveliness_duration, self._liveliness_check, reset=True
        )
        self._is_shutdown = False
        try:
            while not rospy.is_shutdown() and not self._is_shutdown:
                ready, _, _ = select.select([self._socket, shutdown_listener_fd], [], [])
                if self._socket in ready:
                    nbytes, (host, port) = self._socket.recvfrom_into(self._buffer, len(self._buffer))
                    if port != self._port or host != self._host:
                        rospy.loginfo('Got a message from %s:%d, ignoring', self._host, self._port)
                        continue
                    self._handle_packet(self._buffer[:nbytes])
                if shutdown_listener_fd in ready:
                    self._is_shutdown = True
        finally:
            self._is_shutdown = True
            liveliness_check_timer.shutdown()
            self._send_packet({'ctrl': 'shutdown'})
            os.close(shutdown_listener_fd)
            os.close(self._shutdown_notifier_fd)
            self._socket.close()

    def shutdown(self):
        if not self._is_shutdown:
            raise RuntimeError('cannot shutdown a driver that is not running')
        self._shutdown_notifier_fd.write('!')

    def is_shutdown(self):
        return self._is_shutdown

    def _liveliness_check(self, event):
        if not self._inbound_traffic:
            rospy.logwarn(
                'No inbound traffic from %s:%d in the last %d seconds',
                self._host, self._port, event.last_duration
            )
        self._inbound_traffic = False
        if not self._outbound_traffic:
            rospy.logdebug(
                'No outbound traffic to %s:%d in the last %d seconds',
                self._host, self._port, event.last_duration
            )
            self._assert_liveliness()
        self._outbound_traffic = False

    def _assert_liveliness(self):
        rospy.logdebug("Asserting liveliness with %s:%d", self._host, self._port)
        self._send_packet(msgpack.packb({}))

    def _send_packet(self, packet):
        rospy.logdebug("Sending a '%s' packet to %s:%d", packet, self._host, self._port)
        assert self._socket.sendto(packet, (self._ip, self._port)) == len(packet)
        self._outbound_traffic = True

    def _handle_packet(self, packet):
        rospy.logdebug("Got '%s' packet from %s:%d", packet, self._host, self._port)
        self.handle(msgpack.unpackb(packet))
        self._inbound_traffic = True

    def _handle_control(self, command):
        if not isinstance(command, (list, tuple)):
            command = tuple(command)
        name, args = command[0], command[1:]
        if name not in self._control_handlers:
            rospy.logwarn("Got an unknown '%!s' control command", name)
            return
        for handler in self._control_handlers[name]:
            handler(*args)

    def _handle_shutdown(self):
        rospy.logdebug("Got a 'shutdown' command from %s:%d", self._host, self._port)
        rospy.logerr("Remote %s:%d end is forcing a shutdown", self._host, self._port)
        self.shutdown()

    def send(self, data):
        if self._is_shutdown:
            raise RuntimeError('cannot use a shutdown driver')
        rospy.logdebug("Sending '%!s' to %s:%d", data, self._host, self._port)
        self._send_packet(msgpack.packb(data))

    def handle(self, data):
        rospy.logdebug("Got '%!s' from %s:%d", data, self._host, self._port)
        if not data:
            # Likely the other end asserted liveliness
            return
        for tag, content in data:
            if tag not in self._handlers:
                rospy.logwarn("Got an unknown '%s' data tag", tag)
                continue
            for callback in self._handlers[tag]:
                callback(content)

    def register_handler(self, tag, callback):
        if tag not in self._handlers:
            self._handlers = []
        self._handlers[tag].append(callback)
