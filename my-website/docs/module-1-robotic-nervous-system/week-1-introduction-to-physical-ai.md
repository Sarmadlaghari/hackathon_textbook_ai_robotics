---
title: Introduction to Physical AI and Sensors
sidebar_label: "Week 1: Introduction to Physical AI and Sensors"
sidebar_position: 1
---

# Introduction to Physical AI and Sensors

## Learning Objectives

By the end of this week, students will be able to:
- Define Physical AI and its applications in robotics
- Identify and describe the main types of sensors used in robotics
- Understand the principles of LIDAR and IMU sensors
- Explain how sensors enable robot perception in real-world environments

## Introduction

This week introduces the fundamental concepts of Physical AI and the essential sensors used in robotics applications. Physical AI refers to the integration of artificial intelligence with physical systems, enabling robots to perceive, reason, and act in the real world.

Physical AI represents a paradigm shift in robotics, where artificial intelligence algorithms are tightly integrated with physical systems to create intelligent machines capable of interacting with the real world. Unlike traditional AI systems that operate purely in digital domains, Physical AI systems must handle the complexities of real-world perception, uncertainty, and physical constraints.

### Key Characteristics of Physical AI

- **Embodied Intelligence**: AI algorithms are designed specifically for physical interaction
- **Real-time Processing**: Systems must respond to environmental changes in real-time
- **Sensor Integration**: Multiple sensor modalities work together to perceive the environment
- **Uncertainty Management**: Systems must handle noisy sensor data and uncertain environments
- **Safety Considerations**: Physical systems must operate safely in human environments

## Theory

### Sensors in Robotics

Robots rely on various sensors to perceive their environment and make informed decisions. The quality and integration of sensor data directly impacts the robot's ability to navigate, interact, and perform tasks effectively.

#### Sensor Categories

1. **Proprioceptive Sensors**: Measure internal robot state (joint angles, motor currents)
2. **Exteroceptive Sensors**: Measure external environment (cameras, LIDAR, IMU)
3. **Interoceptive Sensors**: Measure internal robot conditions (temperature, power)

### LIDAR Sensors

Light Detection and Ranging (LIDAR) sensors are optical remote sensing devices that measure properties of scattered light to determine the range of distant objects. LIDAR sensors emit laser pulses and measure the time it takes for the light to return after reflecting off objects. This enables precise distance measurements and the creation of detailed 3D maps of the environment.

#### LIDAR Working Principles

LIDAR operates on the principle of time-of-flight measurement:

1. **Emission**: The sensor emits a laser pulse at the speed of light (c ≈ 3×10⁸ m/s)
2. **Reflection**: The pulse reflects off objects in the environment
3. **Detection**: The sensor detects the returning pulse
4. **Calculation**: Distance is calculated using the formula: distance = (speed of light × time delay) / 2

The factor of 2 accounts for the round trip of the laser pulse.

#### Types of LIDAR Sensors

- **Mechanical LIDAR**: Rotating mirrors to scan the environment (e.g., Velodyne HDL-64)
- **Solid-state LIDAR**: No moving parts, using optical phased arrays or flash LIDAR
- **Coherent LIDAR**: Uses frequency modulation for velocity measurements
- **Direct Detection LIDAR**: Measures only the intensity of returned light

#### LIDAR Specifications and Parameters

- **Range**: Detection distance (typically 10-300m)
- **Accuracy**: Measurement precision (typically 1-3cm)
- **Resolution**: Angular resolution between measurements (0.1°-0.5°)
- **Field of View**: Angular coverage (horizontal and vertical)
- **Scan Rate**: Frequency of complete scans (5-20Hz)
- **Data Rate**: Points generated per second (thousands to millions)

#### LIDAR Applications in Robotics

- Environment mapping and localization (SLAM)
- Obstacle detection and collision avoidance
- 3D scene reconstruction and modeling
- Navigation and path planning
- Object detection and classification
- Precision agriculture and autonomous vehicles

#### Advantages and Limitations

**Advantages:**
- High accuracy and precision
- Works in various lighting conditions
- Provides dense 3D point cloud data
- Effective for distance measurement

**Limitations:**
- Performance affected by weather (fog, rain, snow)
- Expensive compared to other sensors
- Can be affected by highly reflective surfaces
- Limited resolution for fine details

### IMU Sensors

Inertial Measurement Units (IMUs) combine accelerometers, gyroscopes, and sometimes magnetometers to measure the robot's orientation, velocity, and gravitational forces.

#### IMU Components

- **Accelerometer**: Measures linear acceleration
- **Gyroscope**: Measures angular velocity
- **Magnetometer**: Measures magnetic field orientation (compass)

#### IMU Applications in Robotics

- Robot pose estimation
- Motion tracking and control
- Stabilization systems
- Dead reckoning navigation

### ROS 2 Integration

The Robot Operating System 2 (ROS 2) provides standardized interfaces for sensor integration, making it easier to work with LIDAR and IMU sensors in robotics applications.

#### Common Sensor Message Types

- `sensor_msgs/LaserScan`: For LIDAR data
- `sensor_msgs/Imu`: For IMU data
- `sensor_msgs/PointCloud2`: For 3D point cloud data

## Code Examples

### LIDAR Data Subscription

Here's a basic ROS 2 Python node that subscribes to LIDAR data:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np

class LidarSubscriber(Node):
    def __init__(self):
        super().__init__('lidar_subscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',  # Topic name - varies by robot/LIDAR
            self.lidar_callback,
            10)  # QoS queue size
        self.subscription  # Prevent unused variable warning

        self.get_logger().info('LIDAR Subscriber node initialized')

    def lidar_callback(self, msg):
        """Process incoming LIDAR data"""
        # Extract key information from the LaserScan message
        ranges = np.array(msg.ranges)
        angle_min = msg.angle_min
        angle_max = msg.angle_max
        angle_increment = msg.angle_increment

        # Filter out invalid measurements (inf or nan)
        valid_ranges = ranges[np.isfinite(ranges)]

        # Find minimum distance (closest obstacle)
        if len(valid_ranges) > 0:
            min_distance = np.min(valid_ranges)
            self.get_logger().info(f'Minimum distance: {min_distance:.2f}m')

        # Calculate distances within a specific angle range (e.g., front of robot)
        front_angle_start = int(len(ranges) / 2 - len(ranges) / 10)  # -10% of FOV
        front_angle_end = int(len(ranges) / 2 + len(ranges) / 10)    # +10% of FOV
        front_distances = ranges[front_angle_start:front_angle_end]
        front_valid = front_distances[np.isfinite(front_distances)]

        if len(front_valid) > 0:
            front_min = np.min(front_valid)
            self.get_logger().info(f'Front minimum distance: {front_min:.2f}m')

def main(args=None):
    rclpy.init(args=args)

    lidar_subscriber = LidarSubscriber()

    try:
        rclpy.spin(lidar_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        lidar_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### IMU Data Processing

Here's a ROS 2 Python node for processing IMU data:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
import math

class ImuSubscriber(Node):
    def __init__(self):
        super().__init__('imu_subscriber')
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data',  # Topic name - varies by IMU setup
            self.imu_callback,
            10)
        self.subscription

        self.get_logger().info('IMU Subscriber node initialized')

    def imu_callback(self, msg):
        """Process incoming IMU data"""
        # Extract orientation (quaternion)
        orientation = msg.orientation
        roll, pitch, yaw = self.quaternion_to_euler(
            orientation.x, orientation.y, orientation.z, orientation.w)

        # Extract angular velocity
        angular_velocity = msg.angular_velocity
        # Extract linear acceleration
        linear_acceleration = msg.linear_acceleration

        # Log key values
        self.get_logger().info(
            f'Orientation - Roll: {math.degrees(roll):.2f}°, '
            f'Pitch: {math.degrees(pitch):.2f}°, '
            f'Yaw: {math.degrees(yaw):.2f}°'
        )
        self.get_logger().info(
            f'Angular Vel - X: {angular_velocity.x:.2f}, '
            f'Y: {angular_velocity.y:.2f}, '
            f'Z: {angular_velocity.z:.2f}'
        )
        self.get_logger().info(
            f'Linear Accel - X: {linear_acceleration.x:.2f}, '
            f'Y: {linear_acceleration.y:.2f}, '
            f'Z: {linear_acceleration.z:.2f}'
        )

    def quaternion_to_euler(self, x, y, z, w):
        """Convert quaternion to Euler angles (roll, pitch, yaw)"""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

def main(args=None):
    rclpy.init(args=args)

    imu_subscriber = ImuSubscriber()

    try:
        rclpy.spin(imu_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        imu_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Basic Sensor Fusion Example

Here's a simple example of combining LIDAR and IMU data:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import Twist
import numpy as np
import math

class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion_node')

        # Subscribe to LIDAR and IMU data
        self.lidar_subscription = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, 10)
        self.imu_subscription = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)

        # Publisher for robot commands
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Initialize sensor data storage
        self.latest_lidar_data = None
        self.latest_imu_data = None

        self.get_logger().info('Sensor Fusion Node initialized')

    def lidar_callback(self, msg):
        """Store latest LIDAR data"""
        self.latest_lidar_data = msg

        # Process LIDAR data for obstacle detection
        if self.process_lidar_for_obstacles():
            self.get_logger().warn('Obstacle detected! Stopping robot.')
            self.stop_robot()

    def imu_callback(self, msg):
        """Store latest IMU data"""
        self.latest_imu_data = msg

        # Process IMU data for orientation
        orientation = msg.orientation
        roll, pitch, yaw = self.quaternion_to_euler(
            orientation.x, orientation.y, orientation.z, orientation.w)

        # Log orientation (could be used for navigation)
        self.get_logger().info(f'Robot orientation: Yaw={math.degrees(yaw):.2f}°')

    def process_lidar_for_obstacles(self):
        """Check if there are obstacles in front of the robot"""
        if self.latest_lidar_data is None:
            return False

        ranges = np.array(self.latest_lidar_data.ranges)
        valid_ranges = ranges[np.isfinite(ranges)]

        if len(valid_ranges) == 0:
            return False

        # Check front-facing range (middle 20% of the scan)
        center_idx = len(ranges) // 2
        front_range = ranges[center_idx - len(ranges)//10:center_idx + len(ranges)//10]
        front_valid = front_range[np.isfinite(front_range)]

        if len(front_valid) > 0:
            min_front_distance = np.min(front_valid)
            # If obstacle is closer than 1 meter, consider it a threat
            return min_front_distance < 1.0

        return False

    def stop_robot(self):
        """Send stop command to robot"""
        stop_cmd = Twist()
        stop_cmd.linear.x = 0.0
        stop_cmd.angular.z = 0.0
        self.cmd_vel_publisher.publish(stop_cmd)

    def quaternion_to_euler(self, x, y, z, w):
        """Convert quaternion to Euler angles"""
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

def main(args=None):
    rclpy.init(args=args)

    sensor_fusion_node = SensorFusionNode()

    try:
        rclpy.spin(sensor_fusion_node)
    except KeyboardInterrupt:
        pass
    finally:
        sensor_fusion_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Visualization of Sensor Data

For visualizing sensor data, you can use RViz2 which comes with ROS 2. Here's a simple example of publishing data for visualization:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import math

class SensorVisualizationNode(Node):
    def __init__(self):
        super().__init__('sensor_visualization_node')

        self.lidar_subscription = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, 10)

        # Publisher for visualization markers
        self.marker_publisher = self.create_publisher(Marker, '/lidar_points', 10)

        self.get_logger().info('Sensor Visualization Node initialized')

    def lidar_callback(self, msg):
        """Convert LIDAR scan to visualization markers"""
        # Create a marker to visualize LIDAR points
        marker = Marker()
        marker.header = msg.header
        marker.ns = "lidar_points"
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD

        # Set marker scale (point size)
        marker.scale.x = 0.05  # Width
        marker.scale.y = 0.05  # Height
        marker.scale.z = 0.05  # Depth

        # Set marker color (red)
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0  # Alpha (opacity)

        # Convert LIDAR ranges to 3D points
        angle = msg.angle_min
        for i, range_val in enumerate(msg.ranges):
            if not math.isinf(range_val) and not math.isnan(range_val):
                # Calculate x, y coordinates from polar coordinates
                x = range_val * math.cos(angle)
                y = range_val * math.sin(angle)

                point = Point()
                point.x = x
                point.y = y
                point.z = 0.0  # Z is 0 for 2D LIDAR

                marker.points.append(point)

            angle += msg.angle_increment

        # Publish the marker
        self.marker_publisher.publish(marker)

def main(args=None):
    rclpy.init(args=args)

    vis_node = SensorVisualizationNode()

    try:
        rclpy.spin(vis_node)
    except KeyboardInterrupt:
        pass
    finally:
        vis_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Exercises

1. Research and compare different LIDAR sensors available in the market (Hokuyo, Velodyne, Ouster, etc.)
2. Investigate the specifications of common IMU sensors (MPU6050, BNO055, etc.)
3. Explore how sensor fusion combines LIDAR and IMU data for improved perception

## References

- Thrun, S., Burgard, W., & Fox, D. (2005). Probabilistic Robotics
- Siciliano, B., & Khatib, O. (2016). Springer Handbook of Robotics
- ROS 2 Documentation: https://docs.ros.org/en/humble/
- LIDAR Sensor Guide: https://www.osrfoundation.org/