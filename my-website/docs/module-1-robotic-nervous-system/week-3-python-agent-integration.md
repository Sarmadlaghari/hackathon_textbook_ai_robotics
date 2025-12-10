---
title: Python Agent Integration with ROS Controllers + URDF Modeling
sidebar_label: "Week 3: Python Agent Integration with ROS Controllers + URDF Modeling"
sidebar_position: 3
---

# Python Agent Integration with ROS Controllers + URDF Modeling

## Learning Objectives

By the end of this week, students will be able to:
- Integrate Python agents with ROS controllers for robot actuation
- Understand the fundamentals of URDF (Unified Robot Description Format) modeling
- Create and manipulate robot models using URDF
- Implement control interfaces between Python agents and robot hardware
- Simulate robot behavior using controller configurations

## Introduction

This week focuses on the integration of Python-based intelligent agents with ROS controllers and the creation of robot models using URDF (Unified Robot Description Format). This represents a critical step in robotics development, bridging high-level AI decision-making with low-level hardware control. Python agents provide the cognitive layer for decision-making, while ROS controllers manage the precise execution of commands on robot hardware.

The combination of Python agents and ROS controllers enables sophisticated robot behaviors, from simple movement commands to complex manipulation tasks. URDF modeling provides the essential robot description that allows controllers to understand the robot's physical structure and kinematic properties.

## Theory

### Python Agent Integration with ROS Controllers

Python agents in robotics are software components that implement decision-making algorithms, planning systems, or AI-based control strategies. These agents must interface with ROS controllers, which handle the low-level hardware control and ensure precise execution of commands. The integration enables sophisticated robot behaviors by combining high-level cognitive capabilities with precise low-level control.

#### Agent-Controller Architecture

The integration between Python agents and ROS controllers typically follows this layered pattern:

1. **Agent Layer**: Implements high-level decision-making, planning, and reasoning using AI algorithms
2. **Interface Layer**: Translates agent decisions into controller commands and handles communication protocols
3. **Controller Layer**: Executes precise hardware control based on received commands
4. **Hardware Layer**: Physical robot components that execute the actions

#### Types of ROS Controllers

ROS 2 supports several types of controllers for different control needs:

- **Joint Trajectory Controller**: Executes complete trajectories with position, velocity, and acceleration
- **Position Controllers**: Simple position control for individual joints
- **Velocity Controllers**: Velocity-based control for smooth motion
- **Effort Controllers**: Direct torque/force control for precise force application
- **Forward Command Controllers**: Forward desired states to hardware interfaces

#### Control Interface Patterns

There are several common patterns for integrating Python agents with ROS controllers:

1. **Direct Command Pattern**: Agent sends immediate commands to controllers
2. **Trajectory Planning Pattern**: Agent plans complete trajectories and sends them to trajectory controllers
3. **Feedback Control Pattern**: Agent uses sensor feedback to adjust control commands
4. **State Machine Pattern**: Agent implements complex behaviors through state transitions

#### Communication Mechanisms

Python agents communicate with controllers using several ROS 2 communication patterns:

- **Topics**: For continuous state publishing and command streaming
- **Services**: For synchronous configuration and immediate actions
- **Actions**: For long-running tasks with progress feedback
- **Parameters**: For dynamic configuration changes

#### Python Agent Implementation Considerations

When implementing Python agents for ROS controller integration, consider:

- **Real-time constraints**: Ensure control loop timing requirements are met
- **Safety boundaries**: Implement safety checks and limits
- **Error handling**: Gracefully handle controller failures and exceptions
- **State management**: Maintain consistent state between agent and controllers
- **Logging and diagnostics**: Track agent decisions and controller responses

#### ros2_control Framework Components

The `ros2_control` framework provides several key components for Python agent integration:

- **Hardware Interface**: Abstracts communication with specific hardware platforms
- **Controller Manager**: Manages the lifecycle of controllers (load, configure, start, stop)
- **Controller Types**: Pre-built controllers for common control tasks
- **Transmission Interface**: Maps actuator commands to joint commands
- **Resource Manager**: Tracks and manages hardware resources

#### Controller Configuration

Controllers are configured using YAML files that specify parameters such as:
- Joint names and mapping
- Control frequency
- Command and state interfaces
- Controller-specific parameters

Example controller configuration:

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

    joint_trajectory_controller:
      type: joint_trajectory_controller/JointTrajectoryController

joint_trajectory_controller:
  ros__parameters:
    joints:
      - joint1
      - joint2
      - joint3
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
```

### URDF (Unified Robot Description Format)

URDF (Unified Robot Description Format) is an XML-based format used to describe robot models in ROS. It defines the robot's physical and kinematic properties, including links, joints, inertial properties, visual and collision models. URDF serves as the foundation for robot simulation, visualization, and kinematic analysis in ROS-based systems.

#### URDF Components and Structure

URDF documents have a hierarchical structure with several key components:

**Links**: Rigid parts of the robot (e.g., base, arms, wheels) that have physical properties:
- **Visual**: How the link appears in simulation and visualization
- **Collision**: How the link interacts with the environment for physics simulation
- **Inertial**: Physical properties like mass, center of mass, and inertia tensor

**Joints**: Connections between links that define the kinematic relationship:
- **Joint Types**: revolute (rotational), prismatic (linear), fixed, continuous, planar, floating
- **Joint Limits**: Position, velocity, and effort limits for safety
- **Joint Origins**: Position and orientation relative to parent link

**Materials**: Color and visual appearance definitions for rendering

#### URDF XML Structure

A basic URDF follows this XML structure:

```xml
<robot name="robot_name">
  <!-- Define materials -->
  <material name="color_name">
    <color rgba="r g b a"/>
  </material>

  <!-- Define links -->
  <link name="link_name">
    <visual>
      <origin xyz="x y z" rpy="roll pitch yaw"/>
      <geometry>
        <!-- Shape definition: box, cylinder, sphere, mesh -->
      </geometry>
      <material name="material_name"/>
    </visual>
    <collision>
      <origin xyz="x y z" rpy="roll pitch yaw"/>
      <geometry>
        <!-- Shape definition -->
      </geometry>
    </collision>
    <inertial>
      <mass value="mass_value"/>
      <origin xyz="x y z" rpy="roll pitch yaw"/>
      <inertia ixx="ixx" ixy="ixy" ixz="ixz" iyy="iyy" iyz="iyz" izz="izz"/>
    </inertial>
  </link>

  <!-- Define joints -->
  <joint name="joint_name" type="joint_type">
    <parent link="parent_link_name"/>
    <child link="child_link_name"/>
    <origin xyz="x y z" rpy="roll pitch yaw"/>
    <axis xyz="x y z"/>  <!-- For revolute and prismatic joints -->
    <limit lower="lower_limit" upper="upper_limit"
           effort="max_effort" velocity="max_velocity"/>
  </joint>
</robot>
```

#### Joint Types in Detail

- **Revolute**: Rotational joint with limited range of motion (like an elbow)
- **Continuous**: Rotational joint without limits (like a wheel)
- **Prismatic**: Linear sliding joint (like a piston)
- **Fixed**: No movement, permanently connects two links
- **Planar**: Movement in a plane
- **Floating**: 6 DOF movement (no constraints)

#### Geometry Types

URDF supports several geometric shapes for visual and collision models:
- **Box**: Defined by size="x_length y_width z_height"
- **Cylinder**: Defined by radius and length
- **Sphere**: Defined by radius
- **Mesh**: Defined by filename and scale (for complex shapes)

#### Inertial Properties

The inertial properties are crucial for accurate physics simulation:
- **Mass**: The mass of the link in kilograms
- **Center of Mass**: The center of mass offset from the link origin
- **Inertia Tensor**: 6 values representing the 3x3 inertia matrix (ixx, ixy, ixz, iyy, iyz, izz)

#### URDF Best Practices

- **Naming Conventions**: Use descriptive, consistent names (e.g., `base_link`, `arm_link1`)
- **Proper Inertial Properties**: Include realistic inertial values for accurate simulation
- **Collision vs Visual**: Use simpler shapes for collision models to improve performance
- **Tree Structure**: Maintain a proper tree structure (no loops in the kinematic chain)
- **Origin Conventions**: Use consistent origin placements (e.g., at joint centers)
- **Xacro Usage**: For complex robots, use Xacro (XML Macros) to parameterize and reuse components

#### URDF Tools and Validation

ROS provides several tools for working with URDF:
- `check_urdf`: Validates URDF syntax and structure
- `urdf_to_graphiz`: Generates visual representation of the kinematic tree
- `robot_state_publisher`: Publishes transforms based on joint states
- `joint_state_publisher`: Publishes default joint states for visualization

#### Xacro for Complex Models

Xacro (XML Macros) extends URDF with features like:
- Variables and constants
- Mathematical expressions
- Macros for reusable components
- Include statements for modular design

Example Xacro usage:
```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="robot_name">
  <!-- Define constants -->
  <xacro:property name="M_PI" value="3.14159"/>

  <!-- Define a macro for repeated components -->
  <xacro:macro name="simple_wheel" params="prefix parent xyz">
    <joint name="${prefix}_wheel_joint" type="continuous">
      <parent link="${parent}"/>
      <child link="${prefix}_wheel_link"/>
      <origin xyz="${xyz}" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
    </joint>

    <link name="${prefix}_wheel_link">
      <visual>
        <geometry>
          <cylinder radius="0.1" length="0.05"/>
        </geometry>
      </visual>
      <collision>
        <geometry>
          <cylinder radius="0.1" length="0.05"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="0.5"/>
        <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.02"/>
      </inertial>
    </link>
  </xacro:macro>

  <!-- Use the macro -->
  <xacro:simple_wheel prefix="front_left" parent="base_link" xyz="0.2 0.1 -0.05"/>
</robot>
```

## Code Examples

### Python Agent with Joint State Subscriber

Here's an example of a Python agent that subscribes to joint states:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from control_msgs.msg import JointTrajectoryControllerState
import numpy as np

class RobotAgentNode(Node):
    def __init__(self):
        super().__init__('robot_agent_node')

        # Subscribe to joint states
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Publisher for joint commands
        self.joint_command_publisher = self.create_publisher(
            Float64MultiArray,
            '/position_commands',
            10
        )

        # Store current joint positions
        self.current_positions = {}

        # Timer for agent decision-making
        self.timer = self.create_timer(0.1, self.agent_decision_callback)

        self.get_logger().info('Robot Agent Node initialized')

    def joint_state_callback(self, msg):
        """Update current joint positions"""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_positions[name] = msg.position[i]

        self.get_logger().info(f'Updated joint positions: {self.current_positions}')

    def agent_decision_callback(self):
        """Make decisions based on current state"""
        # Example: Simple joint position control
        if len(self.current_positions) > 0:
            # Create target positions (example: slightly modify current positions)
            target_positions = []
            for joint_name, current_pos in self.current_positions.items():
                # Add small adjustment based on some logic
                target_pos = current_pos + 0.01  # Example adjustment
                target_positions.append(target_pos)

            # Publish command
            command_msg = Float64MultiArray()
            command_msg.data = target_positions
            self.joint_command_publisher.publish(command_msg)

            self.get_logger().info(f'Published joint commands: {target_positions}')

def main(args=None):
    rclpy.init(args=args)

    robot_agent_node = RobotAgentNode()

    try:
        rclpy.spin(robot_agent_node)
    except KeyboardInterrupt:
        pass
    finally:
        robot_agent_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Controller Interface Node

Here's an example of a Python node that interfaces with ROS controllers:

```python
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time

class ControllerInterfaceNode(Node):
    def __init__(self):
        super().__init__('controller_interface_node')

        # Publisher for trajectory commands
        self.trajectory_publisher = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        # Timer to send trajectory commands
        self.timer = self.create_timer(2.0, self.send_trajectory_command)

        self.get_logger().info('Controller Interface Node initialized')

    def send_trajectory_command(self):
        """Send a trajectory command to the controller"""
        trajectory_msg = JointTrajectory()

        # Define joint names (example for a simple robot)
        trajectory_msg.joint_names = ['joint1', 'joint2', 'joint3']

        # Create trajectory point
        point = JointTrajectoryPoint()

        # Set positions for each joint
        point.positions = [0.5, 1.0, -0.5]  # Example positions in radians

        # Set velocities (optional)
        point.velocities = [0.0, 0.0, 0.0]

        # Set accelerations (optional)
        point.accelerations = [0.0, 0.0, 0.0]

        # Set time from start
        point.time_from_start = Duration(sec=1, nanosec=0)

        # Add point to trajectory
        trajectory_msg.points = [point]

        # Publish trajectory
        self.trajectory_publisher.publish(trajectory_msg)

        self.get_logger().info(f'Published trajectory command: {point.positions}')

def main(args=None):
    rclpy.init(args=args)

    controller_interface_node = ControllerInterfaceNode()

    try:
        rclpy.spin(controller_interface_node)
    except KeyboardInterrupt:
        pass
    finally:
        controller_interface_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Advanced Python Agent with Action Client for Trajectory Execution

Here's an example of a more advanced Python agent using actions for trajectory execution:

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time
import math

class AdvancedRobotAgentNode(Node):
    def __init__(self):
        super().__init__('advanced_robot_agent_node')

        # Create action client for trajectory execution
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            'joint_trajectory_controller/follow_joint_trajectory'
        )

        # Timer for agent decision-making
        self.timer = self.create_timer(5.0, self.execute_trajectory_callback)

        self.get_logger().info('Advanced Robot Agent Node initialized')

    def execute_trajectory_callback(self):
        """Execute a planned trajectory using action client"""
        goal_msg = FollowJointTrajectory.Goal()

        # Define joint names
        goal_msg.trajectory.joint_names = ['joint1', 'joint2', 'joint3']

        # Create trajectory points
        point1 = JointTrajectoryPoint()
        point1.positions = [0.0, 0.0, 0.0]
        point1.velocities = [0.0, 0.0, 0.0]
        point1.time_from_start = Duration(sec=1, nanosec=0)

        point2 = JointTrajectoryPoint()
        point2.positions = [0.5, 0.5, 0.5]
        point2.velocities = [0.0, 0.0, 0.0]
        point2.time_from_start = Duration(sec=2, nanosec=0)

        point3 = JointTrajectoryPoint()
        point3.positions = [0.0, 0.0, 0.0]  # Return to start
        point3.velocities = [0.0, 0.0, 0.0]
        point3.time_from_start = Duration(sec=3, nanosec=0)

        goal_msg.trajectory.points = [point1, point2, point3]

        self._send_goal_async(goal_msg)

    def _send_goal_async(self, goal_msg):
        """Send goal to trajectory controller"""
        self.get_logger().info('Waiting for action server...')

        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available')
            return

        future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle goal response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')

        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Handle result of trajectory execution"""
        result = future.result().result
        self.get_logger().info(f'Trajectory execution result: {result.error_code}')

    def feedback_callback(self, feedback_msg):
        """Handle feedback during trajectory execution"""
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Trajectory progress: {feedback.joint_names}')

def main(args=None):
    rclpy.init(args=args)

    advanced_agent_node = AdvancedRobotAgentNode()

    try:
        rclpy.spin(advanced_agent_node)
    except KeyboardInterrupt:
        pass
    finally:
        advanced_agent_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Python Agent with Controller State Monitoring

Here's an example of a Python agent that monitors controller states:

```python
import rclpy
from rclpy.node import Node
from control_msgs.msg import JointTrajectoryControllerState
from controller_manager_msgs.srv import ListControllers
import time

class ControllerMonitoringAgent(Node):
    def __init__(self):
        super().__init__('controller_monitoring_agent')

        # Subscribe to controller state
        self.controller_state_subscriber = self.create_subscription(
            JointTrajectoryControllerState,
            '/joint_trajectory_controller/state',
            self.controller_state_callback,
            10
        )

        # Create service client to list controllers
        self.list_controllers_client = self.create_client(
            ListControllers,
            '/controller_manager/list_controllers'
        )

        # Timer for controller monitoring
        self.timer = self.create_timer(1.0, self.monitor_controllers)

        self.get_logger().info('Controller Monitoring Agent initialized')

    def controller_state_callback(self, msg):
        """Handle controller state updates"""
        self.get_logger().info(f'Controller state received')
        self.get_logger().info(f'Joint names: {msg.joint_names}')
        self.get_logger().info(f'Position: {msg.feedback.positions}')
        self.get_logger().info(f'Velocity: {msg.feedback.velocities}')
        self.get_logger().info(f'Effort: {msg.feedback.effort}')

    def monitor_controllers(self):
        """Monitor controller status"""
        if self.list_controllers_client.service_is_ready():
            request = ListControllers.Request()
            future = self.list_controllers_client.call_async(request)
            future.add_done_callback(self.controllers_list_callback)

    def controllers_list_callback(self, future):
        """Handle controllers list response"""
        try:
            response = future.result()
            for controller in response.controller:
                self.get_logger().info(
                    f'Controller: {controller.name}, '
                    f'State: {controller.state}, '
                    f'Type: {controller.type}'
                )
        except Exception as e:
            self.get_logger().error(f'Error getting controllers list: {e}')

def main(args=None):
    rclpy.init(args=args)

    controller_agent = ControllerMonitoringAgent()

    try:
        rclpy.spin(controller_agent)
    except KeyboardInterrupt:
        pass
    finally:
        controller_agent.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### URDF Example - Simple Robot Arm

Here's an example URDF file for a simple robot arm:

```xml
<?xml version="1.0"?>
<robot name="simple_robot_arm" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Materials -->
  <material name="black">
    <color rgba="0.0 0.0 0.0 1.0"/>
  </material>
  <material name="red">
    <color rgba="0.8 0.0 0.0 1.0"/>
  </material>
  <material name="blue">
    <color rgba="0.0 0.0 0.8 1.0"/>
  </material>

  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.2"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.1" radius="0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <!-- First Joint and Link -->
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0.0 0.0 0.05" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="100" velocity="1.0"/>
  </joint>

  <link name="link1">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.3"/>
      </geometry>
      <material name="red"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.05 0.05 0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Second Joint and Link -->
  <joint name="joint2" type="revolute">
    <parent link="link1"/>
    <child link="link2"/>
    <origin xyz="0.0 0.0 0.15" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
  </joint>

  <link name="link2">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.2"/>
      </geometry>
      <material name="black"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.05 0.05 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.3"/>
      <inertia ixx="0.0005" ixy="0.0" ixz="0.0" iyy="0.0005" iyz="0.0" izz="0.0005"/>
    </inertial>
  </link>

  <!-- Third Joint and Link -->
  <joint name="joint3" type="revolute">
    <parent link="link2"/>
    <child link="end_effector"/>
    <origin xyz="0.0 0.0 0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="100" velocity="1.0"/>
  </joint>

  <link name="end_effector">
    <visual>
      <geometry>
        <sphere radius="0.02"/>
      </geometry>
      <material name="red"/>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.02"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
    </inertial>
  </link>
</robot>
```

### Python URDF Parser Example

Here's an example of how to work with URDF in Python:

```python
import rclpy
from rclpy.node import Node
import xml.etree.ElementTree as ET

class URDFAnalyzerNode(Node):
    def __init__(self):
        super().__init__('urdf_analyzer_node')

        self.get_logger().info('URDF Analyzer Node initialized')

        # Example URDF content (in practice, this would be loaded from a file)
        self.example_urdf = '''<?xml version="1.0"?>
<robot name="simple_robot_arm">
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.2"/>
      </geometry>
    </visual>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0.0 0.0 0.05" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="100" velocity="1.0"/>
  </joint>
  <link name="link1">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.3"/>
      </geometry>
    </visual>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>
</robot>'''

        # Parse the URDF
        self.parse_urdf()

    def parse_urdf(self):
        """Parse the URDF and extract information"""
        try:
            root = ET.fromstring(self.example_urdf)

            robot_name = root.attrib.get('name', 'unknown')
            self.get_logger().info(f'Robot name: {robot_name}')

            # Count links and joints
            links = root.findall('link')
            joints = root.findall('joint')

            self.get_logger().info(f'Number of links: {len(links)}')
            self.get_logger().info(f'Number of joints: {len(joints)}')

            # Print link names
            for link in links:
                link_name = link.attrib.get('name')
                self.get_logger().info(f'Link: {link_name}')

            # Print joint names and types
            for joint in joints:
                joint_name = joint.attrib.get('name')
                joint_type = joint.attrib.get('type')
                self.get_logger().info(f'Joint: {joint_name}, Type: {joint_type}')

        except ET.ParseError as e:
            self.get_logger().error(f'Error parsing URDF: {e}')

def main(args=None):
    rclpy.init(args=args)

    urdf_analyzer_node = URDFAnalyzerNode()

    try:
        # Process URDF once
        rclpy.spin_once(urdf_analyzer_node, timeout_sec=1.0)
    except KeyboardInterrupt:
        pass
    finally:
        urdf_analyzer_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Loading URDF from File and Working with Robot Description

Here's an example of how to load a URDF file and work with the robot_description parameter in ROS 2:

```python
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterType
import xml.etree.ElementTree as ET

class URDFLoaderNode(Node):
    def __init__(self):
        super().__init__('urdf_loader_node')

        # Declare parameter for URDF file path
        self.declare_parameter('urdf_file_path', '/path/to/robot.urdf')

        # Get URDF file path from parameters
        self.urdf_file_path = self.get_parameter('urdf_file_path').value

        self.get_logger().info(f'Loading URDF from: {self.urdf_file_path}')

        # Load and parse URDF
        self.robot_description = self.load_urdf_file(self.urdf_file_path)

        if self.robot_description:
            # Set robot_description parameter for other nodes to use
            robot_desc_param = rclpy.Parameter(
                'robot_description',
                ParameterType.PARAMETER_STRING,
                self.robot_description
            )
            self.set_parameters([robot_desc_param])

            self.get_logger().info('URDF loaded and robot_description parameter set')

            # Analyze the URDF structure
            self.analyze_urdf(self.robot_description)
        else:
            self.get_logger().error('Failed to load URDF file')

    def load_urdf_file(self, file_path):
        """Load URDF content from file"""
        try:
            with open(file_path, 'r') as file:
                urdf_content = file.read()
            return urdf_content
        except FileNotFoundError:
            self.get_logger().error(f'URDF file not found: {file_path}')
            return None
        except Exception as e:
            self.get_logger().error(f'Error reading URDF file: {e}')
            return None

    def analyze_urdf(self, urdf_string):
        """Analyze URDF structure and extract key information"""
        try:
            root = ET.fromstring(urdf_string)

            # Extract robot name
            robot_name = root.attrib.get('name', 'unknown_robot')
            self.get_logger().info(f'Robot Name: {robot_name}')

            # Extract all links
            links = root.findall('link')
            self.get_logger().info(f'Total Links: {len(links)}')

            for link in links:
                link_name = link.attrib.get('name')
                # Check for visual, collision, and inertial elements
                has_visual = len(link.findall('visual')) > 0
                has_collision = len(link.findall('collision')) > 0
                has_inertial = len(link.findall('inertial')) > 0

                self.get_logger().info(
                    f'  Link: {link_name}, '
                    f'Visual: {has_visual}, '
                    f'Collision: {has_collision}, '
                    f'Inertial: {has_inertial}'
                )

            # Extract all joints
            joints = root.findall('joint')
            self.get_logger().info(f'Total Joints: {len(joints)}')

            for joint in joints:
                joint_name = joint.attrib.get('name')
                joint_type = joint.attrib.get('type')
                parent_link = joint.find('parent').attrib.get('link') if joint.find('parent') is not None else 'None'
                child_link = joint.find('child').attrib.get('link') if joint.find('child') is not None else 'None'

                self.get_logger().info(
                    f'  Joint: {joint_name}, '
                    f'Type: {joint_type}, '
                    f'Parent: {parent_link}, '
                    f'Child: {child_link}'
                )

        except ET.ParseError as e:
            self.get_logger().error(f'Error parsing URDF: {e}')

def main(args=None):
    rclpy.init(args=args)

    urdf_loader_node = URDFLoaderNode()

    try:
        # Keep the node running to maintain the robot_description parameter
        rclpy.spin(urdf_loader_node)
    except KeyboardInterrupt:
        pass
    finally:
        urdf_loader_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Working with robot_state_publisher and URDF

Here's an example of how to work with robot_state_publisher to publish transforms based on URDF:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math

class RobotStatePublisherNode(Node):
    def __init__(self):
        super().__init__('robot_state_publisher')

        # Subscribe to joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )

        # Create transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Store joint states
        self.joint_states = {}

        # Timer for publishing transforms
        self.timer = self.create_timer(0.05, self.publish_transforms)  # 20 Hz

        self.get_logger().info('Robot State Publisher initialized')

    def joint_state_callback(self, msg):
        """Update joint states from JointState message"""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.joint_states[name] = msg.position[i]

    def publish_transforms(self):
        """Publish transforms based on joint states and URDF structure"""
        # This is a simplified example - in practice, you would calculate
        # transforms based on the URDF kinematic chain

        # Example: Publish transform for a simple 3-DOF arm
        transforms = []

        # Base to Link 1 transform (assuming fixed)
        t1 = TransformStamped()
        t1.header.stamp = self.get_clock().now().to_msg()
        t1.header.frame_id = 'base_link'
        t1.child_frame_id = 'link1'
        t1.transform.translation.x = 0.0
        t1.transform.translation.y = 0.0
        t1.transform.translation.z = 0.05  # Height of first joint
        t1.transform.rotation.x = 0.0
        t1.transform.rotation.y = 0.0
        t1.transform.rotation.z = 0.0
        t1.transform.rotation.w = 1.0
        transforms.append(t1)

        # Link 1 to Link 2 transform (with joint1 rotation)
        if 'joint1' in self.joint_states:
            joint1_pos = self.joint_states['joint1']

            t2 = TransformStamped()
            t2.header.stamp = self.get_clock().now().to_msg()
            t2.header.frame_id = 'link1'
            t2.child_frame_id = 'link2'
            t2.transform.translation.x = 0.0  # Length of link1 along z-axis
            t2.transform.translation.y = 0.0
            t2.transform.translation.z = 0.15  # Length of link1
            # Apply rotation around Z-axis for joint1
            t2.transform.rotation.x = 0.0
            t2.transform.rotation.y = 0.0
            t2.transform.rotation.z = math.sin(joint1_pos / 2.0)
            t2.transform.rotation.w = math.cos(joint1_pos / 2.0)
            transforms.append(t2)

        # Broadcast all transforms
        for transform in transforms:
            self.tf_broadcaster.sendTransform(transform)

def main(args=None):
    rclpy.init(args=args)

    robot_state_publisher = RobotStatePublisherNode()

    try:
        rclpy.spin(robot_state_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        robot_state_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### URDF with ros2_control Integration

Here's an example of how to integrate URDF with ros2_control for hardware interface:

```xml
<?xml version="1.0"?>
<robot name="robot_with_control" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Import ros2_control xacro macros -->
  <xacro:include filename="$(find my_robot_description)/urdf/my_robot.ros2_control.xacro"/>

  <!-- Define materials -->
  <material name="black">
    <color rgba="0.0 0.0 0.0 1.0"/>
  </material>
  <material name="red">
    <color rgba="0.8 0.0 0.0 1.0"/>
  </material>
  <material name="blue">
    <color rgba="0.0 0.0 0.8 1.0"/>
  </material>

  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.2"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.1" radius="0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Joint 1 -->
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0.0 0.0 0.05" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="100" velocity="1.0"/>
  </joint>

  <link name="link1">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.3"/>
      </geometry>
      <material name="red"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.05 0.05 0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- ros2_control hardware interface -->
  <ros2_control name="GazeboSystem" type="system">
    <hardware>
      <plugin>gazebo_ros2_control/GazeboSystem</plugin>
    </hardware>
    <joint name="joint1">
      <command_interface name="position">
        <param name="min">-3.14</param>
        <param name="max">3.14</param>
      </command_interface>
      <command_interface name="velocity">
        <param name="min">-1.0</param>
        <param name="max">1.0</param>
      </command_interface>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
  </ros2_control>

  <!-- Controller manager configuration -->
  <gazebo>
    <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control::GazeboROS2ControlPlugin">
      <parameters>$(find my_robot_bringup)/config/controllers.yaml</parameters>
    </plugin>
  </gazebo>
</robot>
```

## Exercises

1. Create a Python agent that moves a simulated robot arm to a specific position
2. Design a URDF model for a simple mobile robot with differential drive
3. Implement a controller interface that handles position, velocity, and effort commands
4. Extend the URDF example to include a gripper at the end effector

## References

- ROS 2 Control Documentation: https://control.ros.org/
- URDF Tutorials: http://wiki.ros.org/urdf/Tutorials
- ROS 2 Controller Manager: https://github.com/ros-controls/ros2_control
- Xacro Documentation: http://wiki.ros.org/xacro