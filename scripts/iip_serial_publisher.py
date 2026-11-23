#!/usr/bin/env python3
import re
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
import serial
from std_msgs.msg import MultiArrayDimension

from invasiveness_evaluation_skin.msg import Int32MultiArrayWithHeader


def get_param(node, name, default):
    node.declare_parameter(name, default)
    return node.get_parameter(name).value


def as_bool(value):
    if isinstance(value, str):
        return value.lower() in ('true', '1', 'yes', 'on')
    return bool(value)


class IIPSerialPublisher(Node):
    def __init__(self):
        super().__init__('iip_serial_publisher')

        self.port = str(get_param(self, 'port', '/dev/ttyACM1'))
        self.baud_rate = int(get_param(self, 'baud_rate', 115200))
        self.read_timeout = float(get_param(self, 'read_timeout', 0.2))
        self.reconnect_delay = float(get_param(self, 'reconnect_delay', 1.0))
        self.num_channels = int(get_param(self, 'num_channels', 14))
        self.frame_id = str(get_param(self, 'frame_id', 'IIP'))
        self.strain_topic = str(get_param(self, 'strain_out', '/phantom_strain'))
        self.force_topic = str(get_param(self, 'force_out', '/phantom_force'))
        self.publish_force = as_bool(get_param(self, 'publish_force', True))

        self.strain_pub = self.create_publisher(
            Int32MultiArrayWithHeader,
            self.strain_topic,
            10,
        )
        self.force_pub = self.create_publisher(
            Int32MultiArrayWithHeader,
            self.force_topic,
            10,
        )

        self._stop_event = threading.Event()
        self._last_parse_warning = 0.0
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

        force_suffix = f' and {self.force_topic}' if self.publish_force else ''
        self.get_logger().info(
            f'Reading IIP Arduino serial: {self.port} at {self.baud_rate} baud '
            f'-> {self.strain_topic}{force_suffix}'
        )

    def destroy_node(self):
        self._stop_event.set()
        if self._thread.is_alive():
            self._thread.join(timeout=1.0)
        super().destroy_node()

    def _read_loop(self):
        while rclpy.ok() and not self._stop_event.is_set():
            try:
                with serial.Serial(
                    self.port,
                    self.baud_rate,
                    timeout=self.read_timeout,
                ) as ser:
                    time.sleep(1.5)
                    ser.reset_input_buffer()
                    self.get_logger().info(f'Connected to {self.port}')
                    while rclpy.ok() and not self._stop_event.is_set():
                        raw_line = ser.readline()
                        if not raw_line:
                            continue
                        line = raw_line.decode('ascii', errors='ignore').strip()
                        if line:
                            self._handle_line(line)
            except serial.SerialException as exc:
                self.get_logger().warn(
                    f'Serial connection failed on {self.port}: {exc}. '
                    f'Retrying in {self.reconnect_delay:.1f} s'
                )
                self._stop_event.wait(self.reconnect_delay)

    def _handle_line(self, line):
        values = [int(match) for match in re.findall(r'-?\d+', line)]
        if len(values) < self.num_channels:
            now = time.monotonic()
            if now - self._last_parse_warning > 2.0:
                self.get_logger().warn(
                    f'Ignoring short serial line with {len(values)} numeric '
                    f'fields: {line[:120]}'
                )
                self._last_parse_warning = now
            return

        strain_values = values[:self.num_channels]
        force_values = values[self.num_channels:self.num_channels + 1]
        stamp = self.get_clock().now().to_msg()

        strain_msg = self._make_msg(
            stamp,
            'strain_raw_values',
            strain_values,
        )
        self.strain_pub.publish(strain_msg)

        if self.publish_force and force_values:
            force_msg = self._make_msg(
                stamp,
                'force_raw_values',
                force_values,
            )
            self.force_pub.publish(force_msg)

    def _make_msg(self, stamp, label, values):
        msg = Int32MultiArrayWithHeader()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.data.layout.dim = [
            MultiArrayDimension(
                label=label,
                size=len(values),
                stride=len(values),
            )
        ]
        msg.data.data = [int(value) for value in values]
        return msg


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.SIGTERM)
    node = IIPSerialPublisher()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
