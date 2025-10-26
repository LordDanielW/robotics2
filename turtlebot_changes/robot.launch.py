#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Darby Lim

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace


def generate_launch_description():
    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
    ROS_DISTRO = os.environ.get('ROS_DISTRO')
    LDS_MODEL = os.environ['LDS_MODEL']
    LDS_LAUNCH_FILE = '/hlds_laser.launch.py'

    namespace = LaunchConfiguration('namespace', default='')

    usb_port = LaunchConfiguration('usb_port', default='/dev/ttyACM0')

    if ROS_DISTRO == 'humble':
        tb3_param_dir = LaunchConfiguration(
            'tb3_param_dir',
            default=os.path.join(
                get_package_share_directory('turtlebot3_bringup'),
                'param',
                ROS_DISTRO,
                TURTLEBOT3_MODEL + '.yaml'))
    else:
        tb3_param_dir = LaunchConfiguration(
            'tb3_param_dir',
            default=os.path.join(
                get_package_share_directory('turtlebot3_bringup'),
                'param',
                TURTLEBOT3_MODEL + '.yaml'))

    if LDS_MODEL == 'LDS-01':
        lidar_pkg_dir = LaunchConfiguration(
            'lidar_pkg_dir',
            default=os.path.join(get_package_share_directory('hls_lfcd_lds_driver'), 'launch'))
    elif LDS_MODEL == 'LDS-02':
        lidar_pkg_dir = LaunchConfiguration(
            'lidar_pkg_dir',
            default=os.path.join(get_package_share_directory('ld08_driver'), 'launch'))
        LDS_LAUNCH_FILE = '/ld08.launch.py'
    elif LDS_MODEL == 'LDS-03':
        lidar_pkg_dir = LaunchConfiguration(
            'lidar_pkg_dir',
            default=os.path.join(get_package_share_directory('coin_d4_driver'), 'launch'))
        LDS_LAUNCH_FILE = '/single_lidar_node.launch.py'
    else:
        lidar_pkg_dir = LaunchConfiguration(
            'lidar_pkg_dir',
            default=os.path.join(get_package_share_directory('hls_lfcd_lds_driver'), 'launch'))

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'usb_port',
            default_value=usb_port,
            description='Connected USB port with OpenCR'),

        DeclareLaunchArgument(
            'tb3_param_dir',
            default_value=tb3_param_dir,
            description='Full path to turtlebot3 parameter file to load'),

        DeclareLaunchArgument(
            'namespace',
            default_value=namespace,
            description='Namespace for nodes'),

        PushRosNamespace(namespace),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [ThisLaunchFileDir(), '/turtlebot3_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time,
                              'namespace': namespace}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([lidar_pkg_dir, LDS_LAUNCH_FILE]),
            launch_arguments={'port': '/dev/ttyUSB0',
                              'frame_id': 'base_scan',
                              'namespace': namespace}.items(),
        ),

        Node(
            package='turtlebot3_node',
            executable='turtlebot3_ros',
            parameters=[
                tb3_param_dir,
                {'namespace': namespace}],
            arguments=['-i', usb_port],
            output='screen'),

        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            parameters=[
                {
                    'camera_calibration_file': os.path.join(
                        get_package_share_directory('turtlebot3_bringup'),
                        'config',
                        'camera_calibration.yaml'
                    ),
                    'camera_info_url': 'file://' + os.path.join(
                        get_package_share_directory('turtlebot3_bringup'),
                        'config',
                        'camera_calibration.yaml'
                    ),
                }
            ],
            remappings=[
                ('/camera_info', '/camera_info'),
            ],
            output='screen'),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '0.0889', '0.01905', '0.15',  # x, y, z translation in meters (3.5", 0.75", 1")
                '-0.5', '0.5', '-0.5', '0.5',   # x, y, z, w quaternion (90° pitch + 270° roll)
                'base_link',                     # parent frame
                'default_cam'                    # child frame
            ],
            output='screen'),

    ])
