# Move to AprilTag Node

## Overview
This ROS2 node enables a robot to autonomously search for, track, and follow an AprilTag while maintaining a target distance of 50 cm.

## Behavior

### State Machine
The node operates as a state machine with three states:

1. **SEARCHING**: Robot rotates in place until it detects an AprilTag
2. **CENTERING**: Robot rotates to center the detected tag in the camera view
3. **FOLLOWING**: Robot continuously tracks and follows the tag, maintaining target distance

### Following Behavior (Main Feature)
Once the tag is centered, the robot enters continuous following mode:

- **Target Distance**: 50 cm (configurable)
- **Deadband**: ±5 cm around target (no movement zone)
- **Forward Movement**: When tag is farther than 55 cm (50 + 5)
- **Backward Movement**: When tag is closer than 45 cm (50 - 5)
- **Lateral Tracking**: Continuously centers the tag horizontally
- **Persistent Tracking**: If tag moves, robot follows it in real-time

### Key Features
- ✅ Automatic tag detection and approach
- ✅ Continuous tracking - robot follows tag movements
- ✅ Bidirectional control - moves forward AND backward
- ✅ Deadband prevents oscillation around target distance
- ✅ Automatic re-search if tag is lost

## Topics

### Subscribed
- `/detections` (apriltag_msgs/AprilTagDetectionArray): AprilTag detections from apriltag_ros

### Published
- `/cmd_vel` (geometry_msgs/Twist): Velocity commands to control the robot

## Parameters

### Control Parameters (adjustable in code)
```python
target_distance = 0.5        # Target distance in meters (50 cm)
deadband_radius = 0.05       # Deadband radius (5 cm)
centering_tolerance = 0.1    # Centering tolerance (normalized)
```

### Velocity Parameters
```python
search_angular_velocity = 0.3    # Rotation speed while searching (rad/s)
centering_angular_gain = 1.0     # Proportional gain for centering
approach_linear_gain = 0.3       # Proportional gain for distance control
max_linear_velocity = 0.2        # Maximum forward/backward speed (m/s)
max_angular_velocity = 0.5       # Maximum rotation speed (rad/s)
```

## Usage

### Run the complete system (recommended):
```bash
ros2 launch matador move_to_apriltag.launch.py
```

This launch file starts:
1. Camera info publisher
2. AprilTag detection node
3. Move to AprilTag node

### Run standalone (requires AprilTag detection to be running separately):
```bash
ros2 run matador move_to_apriltag
```

## Building

```bash
cd ~/ros2_ws
colcon build --packages-select matador
source install/setup.bash
```

## Algorithm Details

### Distance Control
Uses proportional control with deadband:
- `distance_error = current_distance - target_distance`
- **If |distance_error| ≤ deadband**: No linear movement
- **If distance_error > deadband**: Move forward proportionally
- **If distance_error < -deadband**: Move backward proportionally

### Centering Control
Uses proportional control on horizontal position:
- Tag position normalized to [-0.5, 0.5] (center = 0)
- `angular_velocity = -gain × x_position`
- Negative sign accounts for camera frame orientation

### State Transitions
```
SEARCHING → CENTERING: When tag detected
CENTERING → FOLLOWING: When tag centered
FOLLOWING → SEARCHING: If tag lost
```

## Dependencies
- rclpy
- geometry_msgs
- apriltag_msgs
- apriltag_ros (for tag detection)

## Notes
- The robot will continuously track the tag even after reaching the target distance
- If the tag moves closer than 45 cm, the robot will move backward
- If the tag moves farther than 55 cm, the robot will move forward
- The deadband (45-55 cm) prevents oscillation and provides stable behavior
- Tag loss triggers automatic return to searching state
