#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from apriltag_msgs.msg import AprilTagDetectionArray
from std_msgs.msg import String
from tf2_ros import TransformListener, Buffer
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import math


class MoveToAprilTag(Node):
    """
    ROS2 node that:
    1. Rotates until it detects an AprilTag
    2. Centers the tag in the camera view
    3. Continuously follows the tag, maintaining 50 cm distance
    4. Moves forward if tag is too far (>50cm + deadband)
    5. Moves backward if tag is too close (<50cm - deadband)
    6. Continuously tracks tag movement in all directions
    """
    
    def __init__(self):
        super().__init__('move_to_apriltag')
        
        # Publisher for velocity commands
        self.vel_publisher = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        
        # Publisher for robot state
        self.state_publisher = self.create_publisher(String, '/robot_state', 10)
        
        # Subscriber for AprilTag detections
        self.tag_subscriber = self.create_subscription(
            AprilTagDetectionArray,
            '/detections',
            self.tag_callback,
            10
        )
        
        # TF2 listener for getting tag pose
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Control timer - runs at 10 Hz
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        # State machine states
        self.SEARCHING = 0
        self.CENTERING = 1
        self.FOLLOWING = 2  # Changed from APPROACHING and ARRIVED
        
        # State names for publishing
        self.state_names = {
            self.SEARCHING: 'SEARCHING',
            self.CENTERING: 'CENTERING',
            self.FOLLOWING: 'FOLLOWING'
        }
        
        self.state = self.SEARCHING
        self.publish_state()  # Publish initial state
        
        # Detection data
        self.latest_detection = None
        self.tag_detected = False
        self.detected_tag_frame = None  # Store the TF frame name of detected tag
        
        # Control parameters
        self.target_distance = 0.5  # 50 cm in meters
        self.deadband_radius = 0.05  # 5 cm deadband (no movement within target ± this)
        self.centering_tolerance = 0.1  # Tolerance for centering (normalized)
        
        # Velocity parameters
        self.search_angular_velocity = 0.3  # rad/s for searching
        self.centering_angular_gain = 1.0  # Proportional gain for centering
        self.approach_linear_gain = 0.3  # Proportional gain for approaching
        self.max_linear_velocity = 0.2  # Maximum forward velocity
        self.max_angular_velocity = 0.5  # Maximum angular velocity
        
        # Camera parameters (normalized coordinates: -0.5 to 0.5)
        self.image_width = 640  # Default, will be updated from detections
        self.image_height = 480  # Default
        
        self.get_logger().info('MoveToAprilTag node initialized')
        self.get_logger().info(f'Target distance: {self.target_distance} m')
        self.get_logger().info(f'Starting search...')

    def tag_callback(self, msg):
        """Process AprilTag detection messages."""
        if len(msg.detections) > 0:
            # Use the first detected tag
            self.latest_detection = msg.detections[0]
            self.tag_detected = True
            # Generate the TF frame name for this tag (format: apriltag_16h5_id_X)
            tag_id = self.latest_detection.id
            tag_family = self.latest_detection.family[3:]
            self.detected_tag_frame = f'apriltag_{tag_family}_id_{tag_id}'
        else:
            self.tag_detected = False
            self.latest_detection = None
            self.detected_tag_frame = None

    def publish_state(self):
        """Publish the current state as a string message."""
        state_msg = String()
        state_msg.data = self.state_names.get(self.state, 'UNKNOWN')
        self.state_publisher.publish(state_msg)

    def get_tag_position_in_image(self):
        """
        Get the tag's position in the image frame.
        Returns (x, y) in normalized coordinates where (0, 0) is center,
        and values range from -0.5 to 0.5
        """
        if self.latest_detection is None:
            return None, None
        
        # Get the center of the tag in pixel coordinates
        center_x = self.latest_detection.centre.x
        center_y = self.latest_detection.centre.y
        
        # Normalize to [-0.5, 0.5] range with (0,0) at image center
        normalized_x = (center_x / self.image_width) - 0.5
        normalized_y = (center_y / self.image_height) - 0.5
        
        return normalized_x, normalized_y

    def get_tag_distance(self):
        """
        Get the distance to the tag from the TF transform.
        Returns distance in meters.
        """
        if not self.tag_detected or self.detected_tag_frame is None:
            return None
        
        try:
            # Look up the transform from camera frame to tag frame
            # The camera frame is typically 'camera' or 'camera_link'
            transform = self.tf_buffer.lookup_transform(
                'default_cam',  # Target frame (camera)
                self.detected_tag_frame,  # Source frame (tag)
                rclpy.time.Time(),  # Latest available transform
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            # Get the translation (position) from the transform
            translation = transform.transform.translation
            
            # Calculate distance
            distance = math.sqrt(translation.x**2 + translation.y**2 + translation.z**2)
            
            return distance
            
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f'Could not get transform to {self.detected_tag_frame}: {e}')
            return None

    def control_loop(self):
        """Main control loop - state machine."""
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        
        if self.state == self.SEARCHING:
            self.search_for_tag(cmd)
            
        elif self.state == self.CENTERING:
            self.center_tag(cmd)
            
        elif self.state == self.FOLLOWING:
            self.follow_tag(cmd)
        
        # Publish velocity command
        self.vel_publisher.publish(cmd)

    def search_for_tag(self, cmd):
        """State: Rotate in place until a tag is detected."""
        if self.tag_detected:
            self.get_logger().info('Tag detected! Transitioning to CENTERING state')
            self.state = self.CENTERING
            self.publish_state()
            cmd.twist.angular.z = 0.0
        else:
            # Continue rotating
            cmd.twist.angular.z = self.search_angular_velocity

    def center_tag(self, cmd):
        """State: Rotate to center the tag in the camera view."""
        if not self.tag_detected:
            self.get_logger().warn('Tag lost! Returning to SEARCHING state')
            self.state = self.SEARCHING
            self.publish_state()
            return
        
        x_pos, _ = self.get_tag_position_in_image()
        
        if x_pos is None:
            return
        
        # Check if tag is centered (x position near 0)
        if abs(x_pos) < self.centering_tolerance:
            distance = self.get_tag_distance()
            if distance is not None:
                self.get_logger().info(f'Tag centered! Distance: {distance:.2f} m')
                self.get_logger().info('Transitioning to FOLLOWING state - will continuously track tag')
                self.state = self.FOLLOWING
                self.publish_state()
            cmd.twist.angular.z = 0.0
        else:
            # Proportional control to center the tag
            # Negative because positive x_pos means tag is to the right,
            # so we need to turn right (negative angular velocity)
            angular_vel = -self.centering_angular_gain * x_pos
            cmd.twist.angular.z = max(min(angular_vel, self.max_angular_velocity), 
                               -self.max_angular_velocity)

    def follow_tag(self, cmd):
        """State: Continuously follow the tag, maintaining target distance with deadband."""
        if not self.tag_detected:
            self.get_logger().warn('Tag lost! Returning to SEARCHING state')
            self.state = self.SEARCHING
            self.publish_state()
            return
        
        distance = self.get_tag_distance()
        x_pos, _ = self.get_tag_position_in_image()
        
        if distance is None or x_pos is None:
            return
        
        # Calculate distance error
        distance_error = distance - self.target_distance
        
        # Check if we're within the deadband (no movement zone)
        if abs(distance_error) <= self.deadband_radius:
            # Within deadband - only maintain centering, no forward/backward movement
            cmd.twist.linear.x = 0.0
        elif distance_error > 0:
            # Tag is too far - move forward
            linear_vel = self.approach_linear_gain * distance_error
            cmd.twist.linear.x = min(linear_vel, self.max_linear_velocity)
        else:
            # Tag is too close - move backward
            linear_vel = self.approach_linear_gain * distance_error
            # Clamp backward velocity (negative)
            cmd.twist.linear.x = max(linear_vel, -self.max_linear_velocity)
        
        # Continuously maintain centering while following
        angular_vel = -self.centering_angular_gain * x_pos * 0.5  # Reduced gain while moving
        cmd.twist.angular.z = max(min(angular_vel, self.max_angular_velocity), 
                           -self.max_angular_velocity)
        
        # Log progress periodically
        if hasattr(self, '_last_log_time'):
            if (self.get_clock().now() - self._last_log_time).nanoseconds > 1e9:  # Every 1 second
                status = "in deadband" if abs(distance_error) <= self.deadband_radius else \
                         "too far" if distance_error > 0 else "too close"
                self.get_logger().info(f'Following... Distance: {distance:.2f} m ({status}), X offset: {x_pos:.2f}')
                self._last_log_time = self.get_clock().now()
        else:
            self._last_log_time = self.get_clock().now()
        
    def stop_robot(self):
        """Send stop command to robot."""
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.twist.linear.x = 0.0
        cmd.twist.angular.z = 0.0
        self.vel_publisher.publish(cmd)
        self.get_logger().info('Robot stopped')


def main(args=None):
    rclpy.init(args=args)
    node = MoveToAprilTag()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt received')
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
