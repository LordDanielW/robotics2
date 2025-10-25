#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
import yaml
import os
from ament_index_python.packages import get_package_share_directory


class CameraInfoPublisher(Node):
    def __init__(self):
        super().__init__('camera_info_publisher')
        
        # Get the path to the calibration file
        package_dir = get_package_share_directory('matador')
        calibration_file = os.path.join(package_dir, 'config', 'camera_calibration.yaml')
        
        # Load calibration data
        self.camera_info = self.load_camera_info(calibration_file)
        
        # Create publisher
        self.publisher = self.create_publisher(CameraInfo, '/camera/info', 10)
        
        # Create timer to publish at 30 Hz
        self.timer = self.create_timer(1.0/30.0, self.publish_camera_info)
        
        self.get_logger().info(f'Camera info publisher started, publishing calibration from: {calibration_file}')

    def load_camera_info(self, calibration_file):
        """Load camera calibration from YAML file."""
        try:
            with open(calibration_file, 'r') as f:
                calib_data = yaml.safe_load(f)
        except FileNotFoundError:
            self.get_logger().error(f'Calibration file not found: {calibration_file}')
            return None
        except Exception as e:
            self.get_logger().error(f'Error loading calibration: {e}')
            return None

        camera_info = CameraInfo()
        
        # Basic info
        camera_info.width = calib_data['image_width']
        camera_info.height = calib_data['image_height']
        camera_info.distortion_model = calib_data['distortion_model']
        
        # Camera matrix (K)
        camera_info.k = calib_data['camera_matrix']['data']
        
        # Distortion coefficients (D)
        camera_info.d = calib_data['distortion_coefficients']['data']
        
        # Rectification matrix (R)
        camera_info.r = calib_data['rectification_matrix']['data']
        
        # Projection matrix (P)
        camera_info.p = calib_data['projection_matrix']['data']
        
        return camera_info

    def publish_camera_info(self):
        """Publish camera info with current timestamp."""
        if self.camera_info is not None:
            self.camera_info.header.stamp = self.get_clock().now().to_msg()
            self.camera_info.header.frame_id = 'default_cam'
            self.publisher.publish(self.camera_info)


def main(args=None):
    rclpy.init(args=args)
    node = CameraInfoPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()