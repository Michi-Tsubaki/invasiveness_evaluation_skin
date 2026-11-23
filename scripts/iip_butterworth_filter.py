#!/usr/bin/env python3
import math
import threading

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from invasiveness_evaluation_skin.msg import Int32MultiArrayWithHeader


def get_param(node, name, default):
    node.declare_parameter(name, default)
    return node.get_parameter(name).value


class ButterworthLowpass:
    def __init__(self, cutoff_hz, sample_rate_hz, num_channels):
        nyquist = 0.5 * sample_rate_hz
        if cutoff_hz <= 0.0:
            raise ValueError('cutoff_hz must be positive')
        if cutoff_hz >= nyquist:
            raise ValueError('cutoff_hz must be lower than Nyquist frequency')

        q = 1.0 / math.sqrt(2.0)
        omega = 2.0 * math.pi * cutoff_hz / sample_rate_hz
        sin_omega = math.sin(omega)
        cos_omega = math.cos(omega)
        alpha = sin_omega / (2.0 * q)

        b0 = (1.0 - cos_omega) / 2.0
        b1 = 1.0 - cos_omega
        b2 = (1.0 - cos_omega) / 2.0
        a0 = 1.0 + alpha
        a1 = -2.0 * cos_omega
        a2 = 1.0 - alpha

        self.b0 = b0 / a0
        self.b1 = b1 / a0
        self.b2 = b2 / a0
        self.a1 = a1 / a0
        self.a2 = a2 / a0

        self.x1 = np.zeros(num_channels, dtype=np.float64)
        self.x2 = np.zeros(num_channels, dtype=np.float64)
        self.y1 = np.zeros(num_channels, dtype=np.float64)
        self.y2 = np.zeros(num_channels, dtype=np.float64)
        self.initialized = False

    def filter(self, x):
        x = np.asarray(x, dtype=np.float64)
        if not self.initialized:
            self.x1 = x.copy()
            self.x2 = x.copy()
            self.y1 = x.copy()
            self.y2 = x.copy()
            self.initialized = True
            return x.copy()

        y = (
            self.b0 * x
            + self.b1 * self.x1
            + self.b2 * self.x2
            - self.a1 * self.y1
            - self.a2 * self.y2
        )

        self.x2 = self.x1
        self.x1 = x
        self.y2 = self.y1
        self.y1 = y
        return y


class IIPButterworthFilterNode(Node):
    def __init__(self):
        super().__init__('iip_butterworth_filter')

        self.strain_in = str(get_param(self, 'strain_in', '/phantom_strain'))
        self.strain_out = str(get_param(self, 'strain_out', '/phantom_strain_filtered'))
        self.sample_rate_hz = float(get_param(self, 'sample_rate_hz', 50.0))
        self.cutoff_hz = float(get_param(self, 'cutoff_hz', 5.0))
        self.num_channels = int(get_param(self, 'num_channels', 14))

        self.lock = threading.Lock()
        self.last_size_warning_time = 0.0
        self.filter = ButterworthLowpass(
            cutoff_hz=self.cutoff_hz,
            sample_rate_hz=self.sample_rate_hz,
            num_channels=self.num_channels,
        )

        self.pub = self.create_publisher(
            Int32MultiArrayWithHeader,
            self.strain_out,
            10,
        )
        self.sub = self.create_subscription(
            Int32MultiArrayWithHeader,
            self.strain_in,
            self.callback,
            10,
        )

        self.get_logger().info(
            'Publishing 2nd-order Butterworth low-pass strain: '
            f'{self.strain_in} -> {self.strain_out} '
            f'(cutoff={self.cutoff_hz:.3f} Hz, '
            f'sample_rate={self.sample_rate_hz:.3f} Hz, '
            f'channels={self.num_channels})'
        )

    def callback(self, msg):
        raw = np.asarray(msg.data.data, dtype=np.float64)
        if raw.size != self.num_channels:
            now = self.get_clock().now().nanoseconds * 1e-9
            if now - self.last_size_warning_time > 2.0:
                self.get_logger().warn(
                    f'Expected {self.num_channels} channels, got {raw.size}. '
                    'Reinitializing filter.'
                )
                self.last_size_warning_time = now
            with self.lock:
                self.num_channels = int(raw.size)
                self.filter = ButterworthLowpass(
                    cutoff_hz=self.cutoff_hz,
                    sample_rate_hz=self.sample_rate_hz,
                    num_channels=self.num_channels,
                )

        with self.lock:
            filtered = self.filter.filter(raw)

        out = Int32MultiArrayWithHeader()
        out.header = msg.header
        out.data.layout = msg.data.layout
        out.data.data = np.rint(filtered).astype(np.int32).tolist()
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = IIPButterworthFilterNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
