---
title: Physics Simulation in Gazebo
sidebar_label: "Week 4: Physics Simulation in Gazebo"
sidebar_position: 1
---

# Physics Simulation in Gazebo

## Learning Objectives

By the end of this week, students will be able to:
- Understand the fundamentals of physics simulation in robotics
- Configure Gazebo for realistic physics simulation
- Implement sensor simulation in Gazebo environments
- Integrate Gazebo with ROS 2 for robot simulation
- Create custom physics worlds and models
- Validate robot behavior in simulated environments

## Introduction

Physics simulation is a critical component of robotics development, allowing engineers to test algorithms, validate robot designs, and train AI systems in a safe, controlled environment. Gazebo has emerged as the leading physics simulation platform for robotics, providing realistic simulation of rigid body dynamics, sensors, and environmental conditions. This week explores the fundamentals of physics simulation in Gazebo and its integration with ROS 2 for comprehensive robot development.

Gazebo provides a sophisticated physics engine that accurately models real-world forces, collisions, and interactions. By simulating these physical properties, researchers can validate robot behaviors before deploying to real hardware, significantly reducing development time and costs. The integration with ROS 2 enables seamless communication between simulated robots and real-world algorithms, creating a powerful development pipeline.

## Theory

### Physics Simulation Fundamentals

Physics simulation in robotics involves modeling the behavior of physical systems using computational methods. The core components include:

#### Rigid Body Dynamics

Rigid body dynamics form the foundation of physics simulation, describing how objects move and interact under the influence of forces. In Gazebo, each simulated object is treated as a rigid body with properties like mass, center of mass, and inertia tensor. The physics engine calculates the motion of these bodies by solving Newton's equations of motion.

The motion of a rigid body is determined by:
- **Linear motion**: F = ma (Force equals mass times acceleration)
- **Angular motion**: τ = Iα (Torque equals moment of inertia times angular acceleration)

Gazebo supports multiple physics engines including ODE (Open Dynamics Engine), Bullet, Simbody, and DART, each with different characteristics for accuracy and performance.

#### Collision Detection and Response

Collision detection algorithms determine when objects intersect or come into contact. Gazebo uses sophisticated algorithms to detect collisions between complex geometries efficiently. Once a collision is detected, the physics engine calculates the appropriate response based on material properties, friction coefficients, and restitution (bounciness).

Collision geometries supported in Gazebo include:
- **Primitive shapes**: Box, sphere, cylinder, capsule
- **Mesh shapes**: Complex geometries defined by triangle meshes
- **Heightmaps**: Terrain defined by height values

#### Friction and Contact Models

Friction models determine how objects interact when in contact. Gazebo implements both static and dynamic friction models, allowing for realistic simulation of various surface interactions. The ODE physics engine uses a contact model that includes parameters for:
- **Mu (μ)**: Primary friction coefficient
- **Mu2**: Secondary friction coefficient (for anisotropic friction)
- **Fdir1**: Direction of the friction force
- **Slip values**: Parameters for slip-based friction models

#### Gravity and Environmental Forces

Gazebo simulates gravitational forces by applying a constant acceleration to all objects in the world. The gravity vector can be customized to simulate different environments (e.g., moon, Mars). Additional environmental forces like wind can be simulated using plugins.

### Gazebo Architecture and Components

Gazebo is built on a modular architecture that separates the physics simulation from the rendering and user interface components:

#### Server Component (gzserver)

The server component handles the physics simulation, sensor simulation, and plugin execution. It runs headlessly and can be controlled through command-line tools or programmatic interfaces. The server manages:

- Physics engine execution
- Sensor data generation
- Model and world state updates
- Plugin loading and execution
- Communication with client interfaces

#### Client Component (gzclient)

The client component provides the graphical user interface for visualizing the simulation. It connects to the server component to display the 3D world and allows user interaction through mouse and keyboard controls. The client handles:

- 3D rendering using OGRE
- User input processing
- Visualization of physics properties
- Camera control and scene management

#### Model Database and World Files

Gazebo uses SDF (Simulation Description Format) files to define models, worlds, and simulation parameters. SDF is an XML-based format similar to URDF but designed specifically for simulation. World files define the complete simulation environment including:

- Initial model positions and states
- Physics engine parameters
- Environmental properties (gravity, atmosphere)
- Lighting and rendering settings

#### Sensor Simulation

Gazebo provides realistic simulation of various sensor types including:
- **Camera sensors**: RGB, depth, stereo cameras
- **LIDAR sensors**: 2D and 3D laser range finders
- **IMU sensors**: Inertial measurement units
- **Force/Torque sensors**: Joint force and torque measurements
- **GPS sensors**: Global positioning simulation
- **Contact sensors**: Collision detection sensors

### SDF (Simulation Description Format)

SDF is the native format for describing simulation worlds in Gazebo. It extends URDF capabilities to include simulation-specific features like physics properties, sensors, and plugins. An SDF file typically contains:

- **World definition**: Environment, gravity, physics engine settings
- **Model definitions**: Robot and object descriptions
- **Light sources**: Lighting configuration
- **Plugins**: Custom simulation logic

Basic SDF structure:
```xml
<sdf version="1.7">
  <world name="default">
    <!-- World properties -->
    <physics type="ode">
      <gravity>0 0 -9.8</gravity>
    </physics>

    <!-- Models -->
    <model name="robot">
      <!-- Model definition -->
    </model>

    <!-- Lights -->
    <light name="sun" type="directional">
      <pose>0 0 10 0 0 0</pose>
    </light>
  </world>
</sdf>
```

### Gazebo-ROS 2 Integration

The integration between Gazebo and ROS 2 is facilitated by the `gazebo_ros_pkgs` package, which provides plugins and tools for seamless communication. Key integration points include:

#### Message Bridges

The integration provides automatic bridges between Gazebo topics and ROS 2 topics:
- `/clock` synchronization for simulation time
- Sensor data publishing to ROS 2 topics
- Actuator command subscription from ROS 2 topics
- Model state publishing and subscription

#### TF Tree Integration

Gazebo automatically publishes transforms for all simulated models, creating a complete TF tree that ROS 2 nodes can use for spatial reasoning.

#### Service Interfaces

The integration provides ROS 2 services for:
- Model spawning and deletion
- World state management
- Simulation control (pause, reset, step)
- Physics parameter adjustment

### Physics Engine Options

Gazebo supports multiple physics engines, each with different characteristics:

#### ODE (Open Dynamics Engine)
- Default physics engine in Gazebo
- Good balance of performance and accuracy
- Supports complex joint types
- Well-tested for robotics applications

#### Bullet
- High-performance physics engine
- Good for real-time simulation
- Supports soft body dynamics
- Used in game development

#### Simbody
- High-fidelity multibody dynamics
- Excellent for biomechanics simulation
- More computationally intensive
- Precise constraint handling

#### DART (Dynamic Animation and Robotics Toolkit)
- Advanced constraint handling
- Support for complex kinematic chains
- Good for humanoid robots
- Hybrid rigid/soft body simulation

### Performance Optimization

Physics simulation can be computationally intensive. Several optimization strategies help maintain performance:

#### Simplified Collision Models

Using simpler geometries for collision detection while maintaining detailed visual models improves performance without sacrificing accuracy.

#### Fixed Time Steps

Using fixed time steps for physics calculations ensures consistent behavior and can improve performance.

#### Level of Detail (LOD)

Implementing multiple levels of detail for complex models allows the simulator to use simpler representations when performance is critical.

## Code Examples

### Basic Gazebo World File

Here's an example of a simple Gazebo world file with physics simulation:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="physics_simulation_example">
    <!-- Physics engine configuration -->
    <physics type="ode">
      <gravity>0 0 -9.8</gravity>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <ode>
        <solver>
          <type>quick</type>
          <iters>10</iters>
          <sor>1.3</sor>
        </solver>
        <constraints>
          <cfm>0.0</cfm>
          <erp>0.2</erp>
          <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
          <contact_surface_layer>0.001</contact_surface_layer>
        </constraints>
      </ode>
    </physics>

    <!-- Include ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Include sky -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Simple box model -->
    <model name="box">
      <pose>0 0 0.5 0 0 0</pose>
      <link name="link">
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.083</ixx>
            <ixy>0.0</ixy>
            <ixz>0.0</ixz>
            <iyy>0.083</iyy>
            <iyz>0.0</iyz>
            <izz>0.083</izz>
          </inertia>
        </inertial>
        <collision name="collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
          <material>
            <ambient>1 0 0 1</ambient>
            <diffuse>1 0 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Falling sphere model -->
    <model name="falling_sphere">
      <pose>1 0 5 0 0 0</pose>
      <link name="link">
        <inertial>
          <mass>0.5</mass>
          <inertia>
            <ixx>0.025</ixx>
            <ixy>0.0</ixy>
            <ixz>0.0</ixz>
            <iyy>0.025</iyy>
            <iyz>0.0</iyz>
            <izz>0.025</izz>
          </inertia>
        </inertial>
        <collision name="collision">
          <geometry>
            <sphere>
              <radius>0.2</radius>
            </sphere>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <sphere>
              <radius>0.2</radius>
            </sphere>
          </geometry>
          <material>
            <ambient>0 0 1 1</ambient>
            <diffuse>0 0 1 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

### Gazebo Plugin Example - Custom Physics Controller

Here's an example of a custom Gazebo plugin for physics control:

```cpp
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Pose3.hh>

namespace gazebo
{
  class PhysicsControllerPlugin : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      // Store the model pointer for later use
      this->model = _model;

      // Get parameters from SDF
      this->target_x = 0.0;
      if (_sdf->HasElement("target_x"))
        this->target_x = _sdf->Get<double>("target_x");

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&PhysicsControllerPlugin::OnUpdate, this));
    }

    // Called by the world update start event
    public: void OnUpdate()
    {
      // Apply a force to move the model toward the target position
      auto current_pose = this->model->WorldPose();
      double current_x = current_pose.Pos().X();

      // Calculate error
      double error = this->target_x - current_x;

      // Apply proportional control force
      ignition::math::Vector3d force(error * 10.0, 0.0, 0.0);
      this->model->SetLinearVel(force * 0.1);
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Target position
    private: double target_x;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(PhysicsControllerPlugin)
}
```

### ROS 2 Node for Gazebo Interaction

Here's an example of a ROS 2 node that interacts with Gazebo simulation:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
from gazebo_msgs.msg import ModelStates
from std_srvs.srv import Empty
import time

class GazeboControllerNode(Node):
    def __init__(self):
        super().__init__('gazebo_controller_node')

        # Create service clients for Gazebo
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.delete_client = self.create_client(DeleteEntity, '/delete_entity')
        self.pause_client = self.create_client(Empty, '/pause_physics')
        self.unpause_client = self.create_client(Empty, '/unpause_physics')
        self.reset_client = self.create_client(Empty, '/reset_simulation')

        # Subscribe to model states
        self.model_states_sub = self.create_subscription(
            ModelStates,
            '/model_states',
            self.model_states_callback,
            10
        )

        # Publisher for robot velocity commands
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Wait for services to be available
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for spawn service...')

        while not self.delete_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for delete service...')

        self.get_logger().info('Gazebo Controller Node initialized')

    def spawn_model(self, model_name, model_xml, robot_namespace=''):
        """Spawn a model in Gazebo"""
        request = SpawnEntity.Request()
        request.name = model_name
        request.xml = model_xml
        request.robot_namespace = robot_namespace

        future = self.spawn_client.call_async(request)
        return future

    def delete_model(self, model_name):
        """Delete a model from Gazebo"""
        request = DeleteEntity.Request()
        request.name = model_name

        future = self.delete_client.call_async(request)
        return future

    def pause_simulation(self):
        """Pause the physics simulation"""
        future = self.pause_client.call_async(Empty.Request())
        return future

    def unpause_simulation(self):
        """Unpause the physics simulation"""
        future = self.unpause_client.call_async(Empty.Request())
        return future

    def reset_simulation(self):
        """Reset the entire simulation"""
        future = self.reset_client.call_async(Empty.Request())
        return future

    def model_states_callback(self, msg):
        """Handle model states updates"""
        for i, name in enumerate(msg.name):
            if name == 'mobile_robot':  # Example robot name
                position = msg.pose[i].position
                velocity = msg.twist[i].linear
                self.get_logger().info(
                    f'Robot {name} position: ({position.x:.2f}, {position.y:.2f}, {position.z:.2f}), '
                    f'velocity: ({velocity.x:.2f}, {velocity.y:.2f}, {velocity.z:.2f})'
                )

    def send_velocity_command(self, linear_x=0.0, angular_z=0.0):
        """Send velocity command to simulated robot"""
        cmd_msg = Twist()
        cmd_msg.linear.x = linear_x
        cmd_msg.angular.z = angular_z
        self.cmd_vel_pub.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)

    gazebo_controller = GazeboControllerNode()

    # Example usage
    try:
        # Send a velocity command
        gazebo_controller.send_velocity_command(linear_x=0.5, angular_z=0.2)

        # Pause for a moment to see the effect
        time.sleep(2.0)

        # Stop the robot
        gazebo_controller.send_velocity_command()

        rclpy.spin_once(gazebo_controller, timeout_sec=1.0)
    except KeyboardInterrupt:
        pass
    finally:
        gazebo_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Physics Parameter Configuration Node

Here's an example of how to configure physics parameters dynamically:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from gazebo_msgs.srv import SetPhysicsProperties, GetPhysicsProperties
from gazebo_msgs.msg import ODEPhysics
from geometry_msgs.msg import Vector3

class PhysicsConfigNode(Node):
    def __init__(self):
        super().__init__('physics_config_node')

        # Create service clients for physics configuration
        self.set_physics_client = self.create_client(
            SetPhysicsProperties,
            '/set_physics_properties'
        )
        self.get_physics_client = self.create_client(
            GetPhysicsProperties,
            '/get_physics_properties'
        )

        # Timer to periodically check and adjust physics properties
        self.timer = self.create_timer(5.0, self.adjust_physics_properties)

        self.get_logger().info('Physics Configuration Node initialized')

    def get_current_physics_properties(self):
        """Get current physics properties from Gazebo"""
        while not self.get_physics_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for get physics service...')

        request = GetPhysicsProperties.Request()
        future = self.get_physics_client.call_async(request)

        # Wait for response
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def set_physics_properties(self, time_step=0.001, max_update_rate=1000.0,
                              gravity=(0, 0, -9.8)):
        """Set physics properties in Gazebo"""
        while not self.set_physics_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for set physics service...')

        request = SetPhysicsProperties.Request()
        request.time_step = time_step
        request.max_update_rate = max_update_rate

        # Set gravity
        gravity_msg = Vector3()
        gravity_msg.x = gravity[0]
        gravity_msg.y = gravity[1]
        gravity_msg.z = gravity[2]
        request.gravity = gravity_msg

        # Set ODE parameters
        request.ode_config.auto_disable_bodies = False
        request.ode_config.sor_pgs_precon_iters = 2
        request.ode_config.sor_pgs_iters = 50
        request.ode_config.sor_pgs_w = 1.3
        request.ode_config.contact_surface_layer = 0.001
        request.ode_config.contact_max_correcting_vel = 100.0
        request.ode_config.cfm = 0.0
        request.ode_config.erp = 0.2
        request.ode_config.max_contacts = 20

        future = self.set_physics_client.call_async(request)
        return future

    def adjust_physics_properties(self):
        """Adjust physics properties based on simulation requirements"""
        # Get current properties
        current_props = self.get_current_physics_properties()

        if current_props:
            self.get_logger().info(f'Current time step: {current_props.time_step}')
            self.get_logger().info(f'Current max update rate: {current_props.max_update_rate}')
            self.get_logger().info(f'Current gravity: {current_props.gravity}')

        # Example: Adjust for higher accuracy (smaller time step)
        # self.set_physics_properties(time_step=0.0005, max_update_rate=2000.0)

        self.get_logger().info('Physics properties checked/adjusted')

def main(args=None):
    rclpy.init(args=args)

    physics_config_node = PhysicsConfigNode()

    try:
        rclpy.spin(physics_config_node)
    except KeyboardInterrupt:
        pass
    finally:
        physics_config_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Advanced Physics Simulation with Custom Contact Detection

Here's an example of a more advanced physics simulation with custom contact detection and response:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ContactsState
from std_msgs.msg import Float64
import math

class AdvancedPhysicsController(Node):
    def __init__(self):
        super().__init__('advanced_physics_controller')

        # Subscribe to contact sensors to detect collisions
        self.contact_sub = self.create_subscription(
            ContactsState,
            '/contact_sensor_state',
            self.contact_callback,
            10
        )

        # Subscribe to laser scan for environment awareness
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/laser_scan',
            self.scan_callback,
            10
        )

        # Publisher for robot velocity commands
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Timer for physics-based control decisions
        self.timer = self.create_timer(0.1, self.physics_control_loop)

        # Physics parameters
        self.collision_detected = False
        self.collision_force = 0.0
        self.scan_ranges = []
        self.safe_distance = 1.0  # meters

        self.get_logger().info('Advanced Physics Controller initialized')

    def contact_callback(self, msg):
        """Handle contact sensor messages"""
        if len(msg.states) > 0:
            self.collision_detected = True
            # Calculate collision force from the first contact
            if len(msg.states[0].wrenches) > 0:
                force = msg.states[0].wrenches[0].force
                self.collision_force = math.sqrt(
                    force.x**2 + force.y**2 + force.z**2
                )
            self.get_logger().info(f'Collision detected with force: {self.collision_force:.2f}')
        else:
            self.collision_detected = False
            self.collision_force = 0.0

    def scan_callback(self, msg):
        """Handle laser scan messages"""
        self.scan_ranges = msg.ranges

    def physics_control_loop(self):
        """Main physics-based control loop"""
        cmd_msg = Twist()

        # Check for obstacles in front
        if self.scan_ranges:
            front_scan = self.scan_ranges[len(self.scan_ranges)//2]  # Front reading
            if not math.isinf(front_scan) and front_scan < self.safe_distance:
                # Slow down when approaching obstacles
                cmd_msg.linear.x = 0.2 * (front_scan / self.safe_distance)
                cmd_msg.angular.z = 0.0
            else:
                # Normal speed when clear
                cmd_msg.linear.x = 0.5
                cmd_msg.angular.z = 0.0
        else:
            # Default forward movement
            cmd_msg.linear.x = 0.3
            cmd_msg.angular.z = 0.0

        # If collision detected, reverse and turn
        if self.collision_detected:
            cmd_msg.linear.x = -0.3  # Move backward
            cmd_msg.angular.z = 0.5  # Turn to avoid obstacle
            self.get_logger().info('Collision response: reversing and turning')

        self.cmd_vel_pub.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)

    physics_controller = AdvancedPhysicsController()

    try:
        rclpy.spin(physics_controller)
    except KeyboardInterrupt:
        pass
    finally:
        physics_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Physics-based Trajectory Planning with Dynamic Obstacles

Here's an example of physics-based trajectory planning that accounts for dynamic obstacles:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from gazebo_msgs.msg import ModelStates
from visualization_msgs.msg import Marker
from std_msgs.msg import Header
import math
import numpy as np
from scipy.spatial.transform import Rotation as R

class PhysicsTrajectoryPlanner(Node):
    def __init__(self):
        super().__init__('physics_trajectory_planner')

        # Subscribe to model states to track all objects in simulation
        self.model_states_sub = self.create_subscription(
            ModelStates,
            '/model_states',
            self.model_states_callback,
            10
        )

        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Publisher for visualization markers
        self.marker_pub = self.create_publisher(
            Marker,
            '/trajectory_visualization',
            10
        )

        # Timer for trajectory planning
        self.timer = self.create_timer(0.2, self.trajectory_planning_loop)

        # Store model states
        self.model_states = {}
        self.robot_name = 'mobile_robot'  # Assuming this is our robot's name

        self.get_logger().info('Physics Trajectory Planner initialized')

    def model_states_callback(self, msg):
        """Update model states from Gazebo"""
        for i, name in enumerate(msg.name):
            pose = msg.pose[i]
            twist = msg.twist[i]

            self.model_states[name] = {
                'position': np.array([pose.position.x, pose.position.y, pose.position.z]),
                'orientation': np.array([pose.orientation.x, pose.orientation.y,
                                        pose.orientation.z, pose.orientation.w]),
                'linear_vel': np.array([twist.linear.x, twist.linear.y, twist.linear.z]),
                'angular_vel': np.array([twist.angular.x, twist.angular.y, twist.angular.z])
            }

    def calculate_collision_probability(self, robot_pos, robot_vel, obstacle_pos, obstacle_vel):
        """Calculate collision probability based on current trajectories"""
        # Simple collision prediction based on relative positions and velocities
        rel_pos = obstacle_pos - robot_pos
        rel_vel = obstacle_vel - robot_vel

        # Calculate time to closest approach
        rel_speed_squared = np.dot(rel_vel, rel_vel)
        if rel_speed_squared < 1e-6:  # Very slow relative motion
            return 0.0

        t_ca = -np.dot(rel_pos, rel_vel) / rel_speed_squared
        t_ca = max(0, t_ca)  # Only look forward in time

        # Calculate closest approach distance
        closest_pos = robot_pos + robot_vel * t_ca
        closest_obstacle_pos = obstacle_pos + obstacle_vel * t_ca
        dist_ca = np.linalg.norm(closest_obstacle_pos - closest_pos)

        # Collision probability based on distance and safety margin
        safety_margin = 0.5  # meters
        if dist_ca < safety_margin:
            return min(1.0, (safety_margin - dist_ca) / safety_margin)
        else:
            return 0.0

    def trajectory_planning_loop(self):
        """Main trajectory planning loop with physics considerations"""
        if self.robot_name not in self.model_states:
            return

        robot_state = self.model_states[self.robot_name]
        robot_pos = robot_state['position']
        robot_vel = robot_state['linear_vel']

        # Check for potential collisions with other models
        collision_risk = 0.0
        avoidance_vector = np.array([0.0, 0.0, 0.0])

        for model_name, model_state in self.model_states.items():
            if model_name == self.robot_name:
                continue  # Skip robot itself

            # Calculate collision probability
            collision_prob = self.calculate_collision_probability(
                robot_pos, robot_vel,
                model_state['position'], model_state['linear_vel']
            )

            if collision_prob > 0.1:  # Significant risk
                collision_risk = max(collision_risk, collision_prob)

                # Calculate avoidance direction (perpendicular to relative velocity)
                rel_pos = model_state['position'] - robot_pos
                rel_vel = model_state['linear_vel'] - robot_vel

                # Create perpendicular vector for avoidance
                avoidance_dir = np.array([-rel_pos[1], rel_pos[0], 0.0])
                if np.linalg.norm(avoidance_dir) > 0:
                    avoidance_dir = avoidance_dir / np.linalg.norm(avoidance_dir)
                    avoidance_vector += avoidance_dir * collision_prob

        # Generate velocity command based on collision risk
        cmd_msg = Twist()

        if collision_risk > 0.3:  # High risk
            # Prioritize avoidance
            cmd_msg.linear.x = 0.2  # Slow down
            cmd_msg.angular.z = avoidance_vector[1] * 2.0  # Turn away from obstacles
        else:
            # Normal navigation
            cmd_msg.linear.x = 0.5  # Move forward
            cmd_msg.angular.z = 0.0  # No turning

        self.cmd_vel_pub.publish(cmd_msg)

        # Publish visualization marker for planned trajectory
        self.publish_trajectory_marker(robot_pos, robot_vel)

    def publish_trajectory_marker(self, robot_pos, robot_vel):
        """Publish visualization marker for the planned trajectory"""
        marker = Marker()
        marker.header = Header()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = 'map'
        marker.ns = 'trajectory'
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        # Set start point (robot position)
        marker.points = []
        start_point = marker.header.stamp  # This will be filled with actual position

        # Calculate end point based on current velocity
        end_pos = robot_pos + robot_vel * 2.0  # 2 seconds ahead

        # Create start and end points
        start_point = PoseStamped().pose.position
        start_point.x = float(robot_pos[0])
        start_point.y = float(robot_pos[1])
        start_point.z = float(robot_pos[2])

        end_point = PoseStamped().pose.position
        end_point.x = float(end_pos[0])
        end_point.y = float(end_pos[1])
        end_point.z = float(end_pos[2])

        marker.points = [start_point, end_point]

        marker.scale.x = 0.1  # Shaft diameter
        marker.scale.y = 0.2  # Head diameter
        marker.color.a = 1.0  # Alpha
        marker.color.r = 0.0  # Red
        marker.color.g = 1.0  # Green
        marker.color.b = 0.0  # Blue

        self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)

    trajectory_planner = PhysicsTrajectoryPlanner()

    try:
        rclpy.spin(trajectory_planner)
    except KeyboardInterrupt:
        pass
    finally:
        trajectory_planner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Exercises

1. Create a Gazebo world with multiple objects and simulate their interactions
2. Implement a custom physics controller plugin for a simulated robot
3. Configure different physics engines and compare their behavior
4. Create a ROS 2 node that dynamically adjusts physics parameters during simulation
5. Simulate a mobile robot navigating through a complex environment with obstacles

## References

- Gazebo Documentation: http://gazebosim.org/
- SDF Format Documentation: http://sdformat.org/
- Gazebo-ROS 2 Integration: https://github.com/ros-simulation/gazebo_ros_pkgs
- Physics Engine Comparison: ODE, Bullet, Simbody, DART documentation