from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    start_serial = LaunchConfiguration('start_serial')
    port = LaunchConfiguration('port')
    baud_rate = LaunchConfiguration('baud_rate')
    raw_strain_in = LaunchConfiguration('raw_strain_in')
    strain_out = LaunchConfiguration('strain_out')
    calib_yaml = LaunchConfiguration('calib_yaml')
    publisher_rate = LaunchConfiguration('publisher_rate')
    frame_id = LaunchConfiguration('frame_id')
    num_channels = LaunchConfiguration('num_channels')

    return LaunchDescription([
        DeclareLaunchArgument('start_serial', default_value='true'),
        DeclareLaunchArgument('port', default_value='/dev/ttyACM2'),
        DeclareLaunchArgument('baud_rate', default_value='115200'),
        DeclareLaunchArgument('frame_id', default_value='IIP'),
        DeclareLaunchArgument('num_channels', default_value='14'),
        DeclareLaunchArgument('raw_strain_in', default_value='/phantom_strain'),
        DeclareLaunchArgument('strain_out', default_value='/iip_strain'),
        DeclareLaunchArgument('calib_yaml', default_value='iip_calib.yaml'),
        DeclareLaunchArgument('publisher_rate', default_value='50.0'),

        Node(
            condition=IfCondition(start_serial),
            package='invasiveness_evaluation_skin',
            executable='iip_serial_publisher.py',
            name='iip_serial_publisher',
            output='screen',
            parameters=[{
                'port': port,
                'baud_rate': baud_rate,
                'frame_id': frame_id,
                'strain_out': raw_strain_in,
                'num_channels': num_channels,
            }],
        ),
        Node(
            package='invasiveness_evaluation_skin',
            executable='iip_publisher.py',
            name='iip_publisher',
            output='screen',
            parameters=[{
                'strain_in': raw_strain_in,
                'strain_out': strain_out,
                'calib_yaml': calib_yaml,
                'rate': publisher_rate,
            }],
        ),
    ])
