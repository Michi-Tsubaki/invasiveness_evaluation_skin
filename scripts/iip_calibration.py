#!/usr/bin/env python3

import os
import threading

from ament_index_python.packages import get_package_share_directory
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
import yaml

from invasiveness_evaluation_skin.msg import Int32MultiArrayWithHeader


def get_param(node, name, default):
    node.declare_parameter(name, default)
    return node.get_parameter(name).value


class IIPCalibration(Node):
    def __init__(self):
        super().__init__('iip_calibration')

        self.strain_topic = str(get_param(self, 'strain_topic', '/phantom_strain'))
        output_path = str(get_param(self, 'yaml_path', 'iip_calib.yaml'))
        if os.path.isabs(output_path):
            self.output_yaml = output_path
        else:
            share_dir = get_package_share_directory('invasiveness_evaluation_skin')
            self.output_yaml = os.path.join(share_dir, 'config', output_path)

        self.strain_baseline_data = []
        self.strain_active_data = []
        self.mode = 'baseline'
        self.is_collecting = True
        self.lock = threading.Lock()

        self.sub = self.create_subscription(
            Int32MultiArrayWithHeader,
            self.strain_topic,
            self.strain_callback,
            10,
        )

        threading.Thread(target=self.wait_for_enter, daemon=True).start()
        self.get_logger().info(
            'Baseline measurement started. Keep static and press Enter when ready.'
        )

    def strain_callback(self, msg):
        with self.lock:
            if not self.is_collecting or len(msg.data.data) != 14:
                return
            if self.mode == 'baseline':
                self.strain_baseline_data.append(list(msg.data.data))
            elif self.mode == 'active':
                self.strain_active_data.append(list(msg.data.data))

    def wait_for_enter(self):
        input()
        with self.lock:
            if self.mode != 'baseline':
                return
            self.mode = 'wait_active'
        self.get_logger().info(
            'Baseline fixed. Press Enter to start max/min acquisition.'
        )
        input()
        with self.lock:
            self.mode = 'active'
        self.get_logger().info('Max/Min acquisition started.')

    def calculate_and_save(self):
        with self.lock:
            baseline_data = list(self.strain_baseline_data)
            active_data = list(self.strain_active_data)

        if not baseline_data or not active_data:
            self.get_logger().warn('No calibration data collected; YAML was not written.')
            return

        strain_base = np.mean(np.array(baseline_data, dtype=float), axis=0)
        strain_active = np.array(active_data, dtype=float)
        strain_max = strain_active.max(axis=0)
        strain_min = strain_active.min(axis=0)
        strain_scale = np.maximum(
            np.abs(strain_max - strain_base),
            np.abs(strain_min - strain_base),
        )
        strain_scale[strain_scale == 0.0] = 1.0

        calib = {
            'strain': {
                'baseline': strain_base.tolist(),
                'scale': strain_scale.tolist(),
            }
        }

        os.makedirs(os.path.dirname(self.output_yaml), exist_ok=True)
        with open(self.output_yaml, 'w', encoding='utf-8') as f:
            yaml.safe_dump(calib, f)
        self.get_logger().info(f'Wrote calibration YAML: {self.output_yaml}')

    def stop_collecting(self):
        with self.lock:
            self.is_collecting = False


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.SIGTERM)
    node = IIPCalibration()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop_collecting()
        node.calculate_and_save()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
