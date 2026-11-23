#!/usr/bin/env python3
import threading
import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA, Float32MultiArray, MultiArrayDimension
from visualization_msgs.msg import Marker, MarkerArray

from invasiveness_evaluation_skin.msg import Int32MultiArrayWithHeader


def get_param(node, name, default):
    node.declare_parameter(name, default)
    return node.get_parameter(name).value


def as_bool(value):
    if isinstance(value, str):
        return value.lower() in ('true', '1', 'yes', 'on')
    return bool(value)


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def jet_color(value, alpha=1.0):
    x = clamp(float(value), 0.0, 1.0)
    red = np.interp(x, [0.00, 0.35, 0.66, 0.89, 1.00], [0.00, 0.00, 1.00, 1.00, 0.50])
    green = np.interp(x, [0.000, 0.125, 0.375, 0.640, 0.910, 1.000], [0.000, 0.000, 1.000, 1.000, 0.000, 0.000])
    blue = np.interp(x, [0.00, 0.11, 0.34, 0.65, 1.00], [0.50, 1.00, 1.00, 0.00, 0.00])
    return ColorRGBA(r=float(red), g=float(green), b=float(blue), a=float(alpha))


class IIPPainVisualizer(Node):
    def __init__(self):
        super().__init__('iip_pain_visualizer')

        self.strain_topic = str(get_param(self, 'strain_in', '/iip_strain'))
        self.marker_topic = str(get_param(self, 'marker_out', '/iip_pain/markers'))
        self.global_topic = str(get_param(self, 'global_out', '/iip_pain/global_values'))
        self.local_topic = str(get_param(self, 'local_out', '/iip_pain/local_values'))
        self.frame_id = str(get_param(self, 'frame_id', 'IIP'))
        self.mode = str(get_param(self, 'mode', 'both'))
        self.baseline_mode = str(get_param(self, 'baseline_mode', 'zero'))
        self.calibration_duration = float(get_param(self, 'calibration_duration', 2.0))
        self.publish_rate = float(get_param(self, 'publish_rate', 20.0))
        self.vmax = float(get_param(self, 'vmax', 2500.0))
        self.height_scale = float(get_param(self, 'height_scale', 0.025))
        self.depth_delta = float(get_param(self, 'depth_delta', 0.003))
        self.use_depth_gradient = as_bool(get_param(self, 'use_depth_gradient', False))
        self.alpha = float(get_param(self, 'alpha', 0.85))

        if self.mode not in ('both', 'global', 'local'):
            self.get_logger().warn(f"Unknown mode '{self.mode}'; using 'both'")
            self.mode = 'both'
        if self.baseline_mode not in ('zero', 'startup'):
            self.get_logger().warn(
                f"Unknown baseline_mode '{self.baseline_mode}'; using 'zero'"
            )
            self.baseline_mode = 'zero'

        self.setup_sensor_positions()

        self.lock = threading.Lock()
        self.latest_msg = None
        self.latest_data = None
        self.latest_seq = 0
        self.processed_seq = 0
        self.calibration_start = None
        self.calibration_data = []
        self.offset_values = None
        self.is_calibrated = self.baseline_mode == 'zero'

        self.last_size_warning_time = 0.0
        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, 1)
        self.global_pub = self.create_publisher(Float32MultiArray, self.global_topic, 1)
        self.local_pub = self.create_publisher(Float32MultiArray, self.local_topic, 1)
        self.sub = self.create_subscription(
            Int32MultiArrayWithHeader,
            self.strain_topic,
            self.callback,
            1,
        )
        self.timer = self.create_timer(
            1.0 / max(self.publish_rate, 1e-6),
            self.timer_callback,
        )

        self.get_logger().info(
            'Publishing IIP pain arrows: '
            f'{self.strain_topic} -> {self.marker_topic}, '
            f'{self.global_topic}, {self.local_topic} '
            f'(mode={self.mode} baseline={self.baseline_mode})'
        )

    def setup_sensor_positions(self):
        r1, z1 = 0.03, -0.003
        r2, z2 = 0.03, -0.006
        angles1 = np.deg2rad([0, 45, 135, 180, 225, 315])
        angles2 = np.deg2rad(np.arange(0, 360, 45))

        self.layer1_positions = np.column_stack([
            -r1 * np.sin(angles1),
            r1 * np.cos(angles1),
            np.full(len(angles1), z1),
        ])
        self.layer2_positions = np.column_stack([
            -r2 * np.sin(angles2),
            r2 * np.cos(angles2),
            np.full(len(angles2), z2),
        ])
        self.layer1_indices = np.asarray([0, 1, 2, 3, 4, 5], dtype=np.int64)
        self.layer2_indices = np.asarray([6, 7, 8, 9, 10, 11, 12, 13], dtype=np.int64)
        self.local_pairs = [
            (0, 6, 0),
            (1, 7, 45),
            (2, 9, 135),
            (3, 10, 180),
            (4, 11, 225),
            (5, 13, 315),
        ]
        self.local_positions = self.layer1_positions.copy()
        self.all_positions = np.vstack([self.layer1_positions, self.layer2_positions])

    def callback(self, msg):
        data = np.asarray(msg.data.data, dtype=np.float32)
        if data.size < 14:
            now = time.monotonic()
            if now - self.last_size_warning_time > 2.0:
                self.get_logger().warn(
                    f'Expected at least 14 strain channels, got {data.size}'
                )
                self.last_size_warning_time = now
            return

        with self.lock:
            if not self.is_calibrated:
                now = time.monotonic()
                if self.calibration_start is None:
                    self.calibration_start = now
                    self.get_logger().info('Pain baseline calibration started')
                if (now - self.calibration_start) < self.calibration_duration:
                    self.calibration_data.append(data)
                    return
                if self.calibration_data:
                    self.offset_values = np.mean(self.calibration_data, axis=0)
                else:
                    self.offset_values = np.zeros_like(data)
                self.is_calibrated = True
                self.get_logger().info('Pain baseline calibration complete')

            self.latest_msg = msg
            self.latest_data = data
            self.latest_seq += 1

    def timer_callback(self):
        with self.lock:
            if self.latest_msg is None or self.latest_seq == self.processed_seq:
                return
            msg = self.latest_msg
            data = self.latest_data.copy()
            seq = self.latest_seq
            offset = self.offset_values.copy() if self.offset_values is not None else np.zeros_like(data)

        values = self.compute_values(data, offset)
        self.global_pub.publish(self.make_array_msg(
            'layer1_mean,layer2_mean,pain_index',
            values['global_values'],
        ))
        self.local_pub.publish(self.make_array_msg(
            'local_pain_0deg,45deg,135deg,180deg,225deg,315deg',
            values['local_values'],
        ))
        self.marker_pub.publish(self.create_markers(msg, values))

        with self.lock:
            self.processed_seq = seq

    def compute_values(self, data, offset):
        strain = np.abs(data - offset)
        layer1 = strain[self.layer1_indices]
        layer2 = strain[self.layer2_indices]
        layer1_mean = float(np.mean(layer1))
        layer2_mean = float(np.mean(layer2))
        global_pain = abs(layer2_mean - layer1_mean)

        local_values = []
        for top_i, bottom_i, _angle in self.local_pairs:
            local_values.append(abs(float(strain[bottom_i]) - float(strain[top_i])))

        if self.use_depth_gradient and self.depth_delta > 0.0:
            global_pain /= self.depth_delta
            local_values = [value / self.depth_delta for value in local_values]

        return {
            'strain': strain,
            'layer1_mean': layer1_mean,
            'layer2_mean': layer2_mean,
            'global_pain': global_pain,
            'global_values': [layer1_mean, layer2_mean, global_pain],
            'local_values': local_values,
        }

    def make_array_msg(self, label, values):
        msg = Float32MultiArray()
        msg.layout.dim = [
            MultiArrayDimension(label=label, size=len(values), stride=len(values))
        ]
        msg.data = [float(v) for v in values]
        return msg

    def create_markers(self, msg, values):
        header = msg.header
        if not header.frame_id:
            header.frame_id = self.frame_id

        out = MarkerArray()
        delete_marker = Marker()
        delete_marker.header = header
        delete_marker.action = Marker.DELETEALL
        out.markers.append(delete_marker)

        marker_id = 1
        marker_id = self.add_gauge_markers(out, header, marker_id)
        if self.mode in ('both', 'global'):
            marker_id = self.add_global_markers(out, header, marker_id, values)
        if self.mode in ('both', 'local'):
            marker_id = self.add_local_markers(out, header, marker_id, values)

        return out

    def add_gauge_markers(self, out, header, marker_id):
        marker = Marker()
        marker.header = header
        marker.ns = 'pain_sensor_positions'
        marker.id = marker_id
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.0025
        marker.scale.y = 0.0025
        marker.scale.z = 0.0025
        marker.color = ColorRGBA(r=0.9, g=0.9, b=0.9, a=0.9)
        marker.points = [
            Point(x=float(x), y=float(y), z=float(z))
            for x, y, z in self.all_positions
        ]
        out.markers.append(marker)
        return marker_id + 1

    def add_global_markers(self, out, header, marker_id, values):
        arrow_len = self.scaled_height(values['global_pain'], self.height_scale)
        marker_id = self.add_arrow(
            out,
            header,
            marker_id,
            'pain_global_arrow',
            Point(x=0.0, y=0.0, z=0.002),
            Point(x=0.0, y=0.0, z=0.002 + arrow_len),
            jet_color(values['global_pain'] / self.vmax, 0.95),
        )
        marker_id = self.add_text(
            out,
            header,
            marker_id,
            'pain_global_value',
            0.0,
            -0.048,
            0.018,
            'Global Pain %.1f' % values['global_pain'],
            0.005,
        )
        return marker_id

    def add_local_markers(self, out, header, marker_id, values):
        local_values = values['local_values']
        for i, value in enumerate(local_values):
            x, y, _z = self.local_positions[i]
            arrow_len = self.scaled_height(value, self.height_scale)
            color = jet_color(value / self.vmax, 0.95)
            marker_id = self.add_arrow(
                out,
                header,
                marker_id,
                'pain_local_arrows',
                Point(x=float(x), y=float(y), z=0.002),
                Point(x=float(x), y=float(y), z=0.002 + arrow_len),
                color,
            )

        return marker_id

    def scaled_height(self, value, max_height):
        if self.vmax <= 0.0:
            return 0.001
        return 0.001 + max_height * clamp(float(value) / self.vmax, 0.0, 1.0)

    def add_arrow(self, out, header, marker_id, ns, start, end, color):
        marker = Marker()
        marker.header = header
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.points = [start, end]
        marker.scale.x = 0.0014
        marker.scale.y = 0.004
        marker.scale.z = 0.006
        marker.color = color
        out.markers.append(marker)
        return marker_id + 1

    def add_text(self, out, header, marker_id, ns, x, y, z, text, size):
        marker = Marker()
        marker.header = header
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)
        marker.pose.position.z = float(z)
        marker.pose.orientation.w = 1.0
        marker.scale.z = float(size)
        marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.95)
        marker.text = text
        out.markers.append(marker)
        return marker_id + 1


def main(args=None):
    rclpy.init(args=args)
    visualizer = IIPPainVisualizer()
    try:
        rclpy.spin(visualizer)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        visualizer.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
