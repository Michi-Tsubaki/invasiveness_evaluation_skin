#!/usr/bin/env python3
import threading

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from scipy.interpolate import Rbf
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from invasiveness_evaluation_skin.msg import Int32MultiArrayWithHeader


def get_param(node, name, default):
    node.declare_parameter(name, default)
    return node.get_parameter(name).value


def jet_colormap(values):
    x = np.clip(values, 0.0, 1.0)
    red = np.interp(
        x,
        [0.00, 0.35, 0.66, 0.89, 1.00],
        [0.00, 0.00, 1.00, 1.00, 0.50],
    )
    green = np.interp(
        x,
        [0.000, 0.125, 0.375, 0.640, 0.910, 1.000],
        [0.000, 0.000, 1.000, 1.000, 0.000, 0.000],
    )
    blue = np.interp(
        x,
        [0.00, 0.11, 0.34, 0.65, 1.00],
        [0.50, 1.00, 1.00, 0.00, 0.00],
    )
    alpha = np.ones_like(x)
    return np.column_stack([red, green, blue, alpha]).astype(np.float32)


def get_bool_param(node, name, default):
    value = get_param(node, name, default)
    if isinstance(value, str):
        return value.lower() in ('true', '1', 'yes', 'on')
    return bool(value)


class Strain3DHeatmapPublisher(Node):
    def __init__(self):
        super().__init__('strain_3d_heatmap_publisher')

        self.strain_topic = str(get_param(self, 'strain_in', '/iip_strain'))
        self.heatmap_topic = str(get_param(self, 'heatmap_out', '/strain_3d_heatmap'))
        self.calibration_duration = float(get_param(self, 'calibration_duration', 2.0))
        self.vmax = float(get_param(self, 'vmax', 500.0))
        self.alpha = float(get_param(self, 'alpha', 0.7))
        self.grid_res = int(get_param(self, 'grid_res', 50))
        self.num_layers = int(get_param(self, 'num_layers', 20))
        self.radius = float(get_param(self, 'radius', 0.04))
        self.z_min = float(get_param(self, 'z_min', -0.008))
        self.z_max = float(get_param(self, 'z_max', 0.0))
        self.publish_rate = float(get_param(self, 'publish_rate', 15.0))
        self.frame_id = str(get_param(self, 'frame_id', 'IIP'))
        self.render_style = str(get_param(self, 'render_style', 'points'))
        self.point_size = float(get_param(
            self,
            'point_size',
            2.0 * self.radius / max(self.grid_res - 1, 1),
        ))
        self.gauge_size = float(get_param(self, 'gauge_size', 0.003))
        self.show_phantom_body = get_bool_param(self, 'show_phantom_body', True)
        self.gauge_circle_radius = float(get_param(self, 'gauge_circle_radius', 0.03))
        gauge_circle_diameter = 2.0 * self.gauge_circle_radius
        self.phantom_length_90deg = float(get_param(self, 'phantom_length_90deg', 4.0 * gauge_circle_diameter))
        self.phantom_length_0deg = float(get_param(self, 'phantom_length_0deg', 3.0 * gauge_circle_diameter))
        self.phantom_thickness = float(get_param(self, 'phantom_thickness', 0.01))
        self.phantom_alpha = float(get_param(self, 'phantom_alpha', 0.7))
        self.phantom_center_x = float(get_param(self, 'phantom_center_x', 0.0))
        self.phantom_center_y = float(get_param(self, 'phantom_center_y', 0.0))
        self.phantom_center_z = float(get_param(self, 'phantom_center_z', -0.005))

        self.lock = threading.Lock()
        self.latest_msg = None
        self.latest_seq = 0
        self.processed_seq = 0
        self.calibration_data = []
        self.offset_values = None
        self.start_time = None
        self.is_calibrated = False

        self.setup_gauge_positions()
        self.setup_grid()
        self.setup_markers()

        self.sub = self.create_subscription(
            Int32MultiArrayWithHeader,
            self.strain_topic,
            self.callback,
            1,
        )
        self.pub = self.create_publisher(MarkerArray, self.heatmap_topic, 1)
        self.timer = self.create_timer(
            1.0 / max(self.publish_rate, 1e-6),
            self.timer_callback,
        )

        self.get_logger().info(
            f'Publishing 3D heatmaps: {self.strain_topic} -> '
            f'{self.heatmap_topic} '
            f'({self.grid_res}x{self.grid_res}x{self.num_layers})'
        )

    def setup_gauge_positions(self):
        r1, z1 = self.gauge_circle_radius, -0.003
        r2, z2 = self.gauge_circle_radius, -0.006

        # Angles are measured counter-clockwise from the +Y axis in the XY plane.
        angles1 = np.deg2rad([0, 45, 135, 180, 225, 315])
        layer1_pos = np.column_stack([
            -r1 * np.sin(angles1),
            r1 * np.cos(angles1),
            np.full(len(angles1), z1),
        ])

        angles2 = np.deg2rad(np.arange(0, 360, 45))
        layer2_pos = np.column_stack([
            -r2 * np.sin(angles2),
            r2 * np.cos(angles2),
            np.full(len(angles2), z2),
        ])

        self.all_positions = np.vstack([layer1_pos, layer2_pos])

    def setup_grid(self):
        self.xi = np.linspace(-self.radius, self.radius, self.grid_res).astype(np.float32)
        self.yi = np.linspace(-self.radius, self.radius, self.grid_res).astype(np.float32)
        self.zi = np.linspace(self.z_min, self.z_max, self.num_layers).astype(np.float32)

        x_grid, y_grid = np.meshgrid(self.xi, self.yi)
        self.xy_mask = (x_grid**2 + y_grid**2) <= self.radius**2

        points = []
        for z_val in self.zi:
            z_grid = np.full_like(x_grid, z_val, dtype=np.float32)
            points.append(np.column_stack([
                x_grid.ravel(),
                y_grid.ravel(),
                z_grid.ravel(),
            ]))
        self.grid_points = np.vstack(points).astype(np.float32)
        self.eval_x = self.grid_points[:, 0]
        self.eval_y = self.grid_points[:, 1]
        self.eval_z = self.grid_points[:, 2]
        self.valid_mask = np.tile(self.xy_mask.ravel(), self.num_layers)
        self.valid_points = self.grid_points[self.valid_mask]
        self.triangle_indices = self.build_triangle_indices()

        if self.render_style == 'surface':
            self.render_points = self.grid_points[self.triangle_indices]
        else:
            if self.render_style != 'points':
                self.get_logger().warn(
                    f"Unknown render_style '{self.render_style}'; using 'points'"
                )
            self.render_style = 'points'
            self.render_points = self.valid_points

        self.render_eval_x = self.render_points[:, 0]
        self.render_eval_y = self.render_points[:, 1]
        self.render_eval_z = self.render_points[:, 2]

    def build_triangle_indices(self):
        nxy = self.grid_res * self.grid_res
        indices = []
        for layer in range(self.num_layers):
            layer_offset = layer * nxy
            for row in range(self.grid_res - 1):
                for col in range(self.grid_res - 1):
                    i00 = row * self.grid_res + col
                    i10 = row * self.grid_res + col + 1
                    i01 = (row + 1) * self.grid_res + col
                    i11 = (row + 1) * self.grid_res + col + 1
                    if not (self.xy_mask.ravel()[[i00, i10, i01, i11]].all()):
                        continue
                    indices.extend([
                        layer_offset + i00,
                        layer_offset + i10,
                        layer_offset + i11,
                        layer_offset + i00,
                        layer_offset + i11,
                        layer_offset + i01,
                    ])
        return np.asarray(indices, dtype=np.int64)

    def setup_markers(self):
        self.heatmap_marker = Marker()
        self.heatmap_marker.ns = 'strain_3d_heatmap'
        self.heatmap_marker.id = 0
        if self.render_style == 'surface':
            self.heatmap_marker.type = Marker.TRIANGLE_LIST
            self.heatmap_marker.scale.x = 1.0
            self.heatmap_marker.scale.y = 1.0
            self.heatmap_marker.scale.z = 1.0
        else:
            self.heatmap_marker.type = Marker.POINTS
            self.heatmap_marker.scale.x = self.point_size
            self.heatmap_marker.scale.y = self.point_size
        self.heatmap_marker.action = Marker.ADD
        self.heatmap_marker.pose.orientation.w = 1.0
        self.heatmap_marker.points = [
            Point(x=float(x), y=float(y), z=float(z))
            for x, y, z in self.render_points
        ]
        self.heatmap_marker.colors = [
            ColorRGBA(r=0.0, g=0.0, b=0.0, a=self.alpha)
            for _ in self.render_points
        ]

        self.gauge_marker = Marker()
        self.gauge_marker.ns = 'strain_gauge_positions'
        self.gauge_marker.id = 1
        self.gauge_marker.type = Marker.SPHERE_LIST
        self.gauge_marker.action = Marker.ADD
        self.gauge_marker.pose.orientation.w = 1.0
        self.gauge_marker.scale.x = self.gauge_size
        self.gauge_marker.scale.y = self.gauge_size
        self.gauge_marker.scale.z = self.gauge_size
        self.gauge_marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
        self.gauge_marker.points = [
            Point(x=float(x), y=float(y), z=float(z))
            for x, y, z in self.all_positions
        ]

        self.phantom_marker = Marker()
        self.phantom_marker.ns = 'phantom_body'
        self.phantom_marker.id = 2
        self.phantom_marker.type = Marker.CUBE
        self.phantom_marker.action = Marker.ADD
        self.phantom_marker.pose.position.x = self.phantom_center_x
        self.phantom_marker.pose.position.y = self.phantom_center_y
        self.phantom_marker.pose.position.z = self.phantom_center_z
        self.phantom_marker.pose.orientation.w = 1.0
        self.phantom_marker.scale.x = self.phantom_length_90deg
        self.phantom_marker.scale.y = self.phantom_length_0deg
        self.phantom_marker.scale.z = self.phantom_thickness
        self.phantom_marker.color = ColorRGBA(
            r=1.0,
            g=0.72,
            b=0.82,
            a=self.phantom_alpha,
        )

    def callback(self, msg):
        data = np.asarray(msg.data.data, dtype=np.float32)
        if data.size < len(self.all_positions):
            self.get_logger().warn('Ignoring strain message with fewer than 14 channels',
                                   throttle_duration_sec=5.0)
            return
        data = data[:len(self.all_positions)]
        now = self.get_clock().now()

        with self.lock:
            if self.start_time is None:
                self.start_time = now
                self.get_logger().info('Calibration started')

            if not self.is_calibrated:
                elapsed = (now - self.start_time).nanoseconds * 1e-9
                if elapsed < self.calibration_duration:
                    self.calibration_data.append(data)
                    return
                if self.calibration_data:
                    self.offset_values = np.mean(self.calibration_data, axis=0)
                else:
                    self.offset_values = np.zeros_like(data)
                self.is_calibrated = True
                self.get_logger().info('Calibration complete')

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
            offset_values = self.offset_values.copy()

        heatmap_msg = self.create_marker_array(msg, data, offset_values)

        with self.lock:
            self.processed_seq = seq
        self.pub.publish(heatmap_msg)

    def create_marker_array(self, msg, data, offset_values):
        x, y, z = self.all_positions.T
        rbf = Rbf(x, y, z, np.abs(data - offset_values), function='multiquadric', smooth=0.1)
        values = rbf(self.render_eval_x, self.render_eval_y, self.render_eval_z).astype(np.float32)

        colors = jet_colormap(values / self.vmax)
        colors[:, 3] = self.alpha

        header = msg.header
        if not header.frame_id:
            header.frame_id = self.frame_id

        self.heatmap_marker.header = header
        for marker_color, color in zip(self.heatmap_marker.colors, colors):
            marker_color.r = float(color[0])
            marker_color.g = float(color[1])
            marker_color.b = float(color[2])
            marker_color.a = float(color[3])
        self.gauge_marker.header = header

        out = MarkerArray()
        markers = []
        if self.show_phantom_body:
            self.phantom_marker.header = header
            markers.append(self.phantom_marker)
        markers.extend([self.heatmap_marker, self.gauge_marker])
        out.markers = markers
        return out


def main(args=None):
    rclpy.init(args=args)
    publisher = Strain3DHeatmapPublisher()
    try:
        rclpy.spin(publisher)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        publisher.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
