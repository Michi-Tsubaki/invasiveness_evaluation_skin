#!/usr/bin/env python3

import os
import threading

from ament_index_python.packages import get_package_share_directory
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import yaml

from invasiveness_evaluation_skin.msg import Int32MultiArrayWithHeader


def get_param(node, name, default):
    node.declare_parameter(name, default)
    return node.get_parameter(name).value


class IIPStrainPublisher(Node):
    def __init__(self):
        super().__init__('iip_strain_publisher')

        self.strain_in = str(get_param(self, 'strain_in', '/phantom_strain'))
        self.strain_out = str(get_param(self, 'strain_out', '/iip_strain'))
        self.calib_yaml = str(get_param(self, 'calib_yaml', 'iip_calib.yaml'))
        self.rate_hz = float(get_param(self, 'rate', 50.0))

        self.strain_base, self.strain_scale = self.load_calibration(self.calib_yaml)
        self.lock = threading.Lock()
        self.last_strain = None

        self.strain_pub = self.create_publisher(
            Int32MultiArrayWithHeader,
            self.strain_out,
            10,
        )
        self.sub = self.create_subscription(
            Int32MultiArrayWithHeader,
            self.strain_in,
            self.strain_callback,
            10,
        )
        self.timer = self.create_timer(
            1.0 / max(self.rate_hz, 1e-6),
            self.publish_latest,
        )

        self.get_logger().info(
            f'Publishing calibrated IIP strain: {self.strain_in} -> '
            f'{self.strain_out} ({self.rate_hz:.1f} Hz)'
        )

    def load_calibration(self, calib_yaml):
        if os.path.isabs(calib_yaml):
            calib_path = calib_yaml
        else:
            share_dir = get_package_share_directory('invasiveness_evaluation_skin')
            calib_path = os.path.join(share_dir, 'config', calib_yaml)

        with open(calib_path, 'r', encoding='utf-8') as f:
            calib = yaml.safe_load(f)

        strain_base = np.array(calib['strain']['baseline'], dtype=float)
        strain_scale = np.array(calib['strain']['scale'], dtype=float)
        strain_scale[strain_scale == 0.0] = 1.0

        self.get_logger().info(f'Loaded calibration: {calib_path}')
        return strain_base, strain_scale

    def strain_callback(self, msg):
        with self.lock:
            self.last_strain = msg

    def publish_latest(self):
        with self.lock:
            if self.last_strain is None:
                return
            last_strain = self.last_strain

        raw = np.array(last_strain.data.data, dtype=float)
        if raw.size != self.strain_base.size:
            self.get_logger().warn(
                f'Calibration has {self.strain_base.size} channels, '
                f'but incoming strain has {raw.size}; skipping sample.'
            )
            return

        norm = (raw - self.strain_base) / self.strain_scale
        out = Int32MultiArrayWithHeader()
        out.header = last_strain.header
        out.data.layout = last_strain.data.layout
        out.data.data = (norm * 1000.0).astype(np.int32).tolist()
        self.strain_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = IIPStrainPublisher()
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
