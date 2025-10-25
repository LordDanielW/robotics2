from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get the path to the config file
    config_dir = os.path.join(
        get_package_share_directory('matador'),
        'config'
    )
    config_file = os.path.join(config_dir, 'apriltag.yaml')

    # Camera info publisher node
    camera_info_publisher_node = Node(
        package='matador',
        executable='calibration_publisher',
        name='camera_info_publisher',
        output='screen'
    )

    # AprilTag node
    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag',
        remappings=[
            ('image_rect', '/image_raw'),
            ('camera_info', '/camera_info'),
        ],
        parameters=[config_file],
        output='screen'
    )

    return LaunchDescription([
        apriltag_node
    ])
