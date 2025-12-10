---
title: ROS 2 Fundamentals — Nodes, Topics, Services, Packages
sidebar_label: "Week 2: ROS 2 Fundamentals — Nodes, Topics, Services, Packages"
sidebar_position: 2
---

# ROS 2 Fundamentals — Nodes, Topics, Services, Packages

## Learning Objectives

By the end of this week, students will be able to:
- Define the core concepts of ROS 2 (Nodes, Topics, Services, Packages)
- Create and run basic ROS 2 nodes
- Understand the publish-subscribe communication pattern
- Implement service-based communication between nodes
- Create and organize ROS 2 packages for robotics applications

## Introduction

This week introduces the fundamental concepts of the Robot Operating System 2 (ROS 2), the middleware framework that enables communication between different components of a robotic system. ROS 2 provides the infrastructure for distributed computing in robotics, allowing different software components to communicate and coordinate with each other seamlessly.

ROS 2 is designed to address the challenges of developing complex robotic systems by providing standardized interfaces, tools, and conventions that simplify the development process. The core concepts of ROS 2 - Nodes, Topics, Services, and Packages - form the foundation for building sophisticated robotic applications.

### Key Components of ROS 2 Architecture

- **Nodes**: Independent processes that perform computation
- **Topics**: Named buses over which nodes exchange messages
- **Services**: Synchronous request/response communication pattern
- **Packages**: Organizational units that contain related functionality
- **Actions**: Goal-oriented communication for long-running tasks
- **Parameters**: Configuration values that can be set at runtime

## Theory

### ROS 2 Nodes

A node is an executable that uses ROS 2 to communicate with other nodes. Nodes are the fundamental building blocks of a ROS 2 system. Each node runs independently and can communicate with other nodes through topics, services, or actions. Nodes encapsulate the computational units of a ROS 2 application and provide the interfaces (publishers, subscribers, services, etc.) that allow the node to interact with other nodes.

#### Node Characteristics

- **Process-based**: Each node runs as a separate process, providing isolation and fault tolerance
- **Communication hub**: Nodes serve as endpoints for ROS 2 communication, managing publishers, subscribers, services, and parameters
- **Resource management**: Nodes manage their own resources including timers, callbacks, and internal state
- **Modularity**: Nodes can be developed, tested, and deployed independently
- **Naming**: Each node has a unique name within the ROS 2 graph that identifies it to other nodes
- **Namespace support**: Nodes can be organized under namespaces for better organization

#### Node Implementation in Python

In Python, ROS 2 nodes are implemented by subclassing the `rclpy.node.Node` class:

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        # Initialize the parent Node class with a node name
        super().__init__('my_node_name')

        # Set parameters, create publishers/subscribers, etc.
        self.get_logger().info('Node initialized successfully')
```

#### Node Lifecycle

ROS 2 nodes follow a defined lifecycle that enables more sophisticated state management, especially for complex robotic applications:

1. **Unconfigured (Primary State)**: Node created but not configured; only basic operations allowed
2. **Inactive (Secondary State)**: Node configured but not active; can communicate but doesn't perform main functions
3. **Active (Secondary State)**: Node is fully operational and performing its intended functions
4. **Finalized (Secondary State)**: Node is shutting down and cleaning up resources

The lifecycle system is optional and primarily used for more complex systems that need sophisticated state management.

#### Node Management Commands

Common command-line tools for working with nodes:

```bash
# List all active nodes
ros2 node list

# Show information about a specific node
ros2 node info <node_name>

# Get the PID of a node
ros2 run lifecycle lifecycle_service_client get_available_states --node-name <node_name>
```

### ROS 2 Topics and Publish-Subscribe Pattern

Topics enable asynchronous communication between nodes using a publish-subscribe pattern. Publishers send messages to topics, and subscribers receive messages from topics. This decouples publishers and subscribers in time and space, allowing for flexible and scalable robot architectures.

#### Topic Communication

- **Decoupled**: Publishers and subscribers don't need to know about each other
- **Asynchronous**: Messages are sent and received independently
- **Broadcast**: One publisher can send to multiple subscribers
- **Typed**: Each topic has a specific message type
- **Named**: Topics have unique names that identify the data stream
- **QoS support**: Quality of Service policies can be configured for different reliability and performance needs

#### Topic Architecture

The publish-subscribe pattern works as follows:
1. A publisher node creates a publisher and sends messages to a named topic
2. A subscriber node creates a subscription to the same named topic
3. The ROS 2 middleware (DDS) handles message delivery between publishers and subscribers
4. Multiple publishers can publish to the same topic, and multiple subscribers can subscribe to the same topic

#### Quality of Service (QoS) Policies

ROS 2 topics support various QoS policies that define how messages are delivered:

- **Reliability**: `RELIABLE` (all messages delivered) or `BEST_EFFORT` (try to deliver messages)
- **Durability**: `TRANSIENT_LOCAL` (store messages for late-joining subscribers) or `VOLATILE` (don't store messages)
- **History**: `KEEP_LAST` (store N most recent messages) or `KEEP_ALL` (store all messages)
- **Depth**: Number of messages to store in history when using `KEEP_LAST`

#### Common Topic Message Types

- `std_msgs`: Basic data types (integers, floats, strings)
- `sensor_msgs`: Sensor data (LIDAR, cameras, IMU)
- `geometry_msgs`: Geometric data (poses, velocities, points)
- `nav_msgs`: Navigation-related messages (paths, occupancy grids)
- `action_msgs`: Action-related messages
- `builtin_interfaces`: Time and duration messages

#### Topic Implementation in Python

Here's how to implement publishers and subscribers in Python:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TopicPublisherNode(Node):
    def __init__(self):
        super().__init__('topic_publisher')
        # Create a publisher with topic name, message type, and QoS queue size
        self.publisher = self.create_publisher(String, 'topic_name', 10)

        # Create a timer to publish messages periodically
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

class TopicSubscriberNode(Node):
    def __init__(self):
        super().__init__('topic_subscriber')
        # Create a subscription to receive messages
        self.subscription = self.create_subscription(
            String,           # Message type
            'topic_name',     # Topic name
            self.listener_callback,  # Callback function
            10                # QoS queue size
        )
        # Prevent unused variable warning
        self.subscription

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

# Example with custom QoS settings
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class QoSTopicNode(Node):
    def __init__(self):
        super().__init__('qos_topic_node')

        # Create a custom QoS profile for reliable communication
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        self.publisher = self.create_publisher(String, 'reliable_topic', qos_profile)
```

#### Topic Management Commands

Common command-line tools for working with topics:

```bash
# List all active topics
ros2 topic list

# Show information about a specific topic
ros2 topic info /topic_name

# Echo messages from a topic
ros2 topic echo /topic_name std_msgs/msg/String

# Publish a message to a topic
ros2 topic pub /topic_name std_msgs/msg/String "data: 'Hello'"

# Show the type of a topic
ros2 topic type /topic_name
```

### ROS 2 Services

Services provide synchronous request/response communication between nodes. A service client sends a request to a service server, which processes the request and returns a response. Services are ideal for operations that have a clear beginning and end, such as configuration changes, computation tasks, or triggering specific actions.

#### Service Characteristics

- **Synchronous**: Client waits for response before continuing
- **Request/Response**: Defined request and response message types
- **Direct communication**: Client communicates directly with server
- **Blocking**: Service calls block until response is received
- **One-to-one**: Each request is handled by a single service server
- **Stateless**: Each service call is independent of others
- **Typed interfaces**: Service definitions specify request and response message types

#### Service Architecture

The service pattern works as follows:
1. A service server node creates a service server and registers it with a service name
2. A service client node creates a service client and connects to the same service name
3. The client sends a request to the server
4. The server processes the request and sends back a response
5. The client receives the response and continues execution

#### Service Implementation in Python

Here's how to implement service servers and clients in Python:

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class ServiceServerNode(Node):
    def __init__(self):
        super().__init__('service_server')
        # Create a service with service name, service type, and callback function
        self.srv = self.create_service(
            AddTwoInts,           # Service type
            'add_two_ints',       # Service name
            self.add_two_ints_callback  # Callback function
        )
        self.get_logger().info('Service server initialized')

    def add_two_ints_callback(self, request, response):
        # Process the request and populate the response
        response.sum = request.a + request.b
        self.get_logger().info(f'Returning {request.a} + {request.b} = {response.sum}')
        return response

class ServiceClientNode(Node):
    def __init__(self):
        super().__init__('service_client')
        # Create a client for the service
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')

        # Wait for the service to be available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        # Populate the request
        self.req.a = a
        self.req.b = b

        # Call the service asynchronously
        self.future = self.cli.call_async(self.req)
        return self.future

# Example of using the client
def main(args=None):
    rclpy.init(args=args)

    client_node = ServiceClientNode()

    # Send request
    future = client_node.send_request(1, 2)

    # Wait for response
    rclpy.spin_until_future_complete(client_node, future)

    # Process response
    if future.result() is not None:
        response = future.result()
        client_node.get_logger().info(f'Result: {response.sum}')
    else:
        client_node.get_logger().error('Exception while calling service: %r' % future.exception())

    client_node.destroy_node()
    rclpy.shutdown()
```

#### Service Definition Files (.srv)

Service definitions are specified in `.srv` files that define the request and response message structure:

```
# Request part (before the '---')
int64 a
int64 b
---
# Response part (after the '---')
int64 sum
```

This defines a service with two integer64 inputs (a, b) and one integer64 output (sum).

#### Service Management Commands

Common command-line tools for working with services:

```bash
# List all active services
ros2 service list

# Show information about a specific service
ros2 service info /service_name

# Call a service from command line
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 1, b: 2}"

# Show the type of a service
ros2 service type /service_name
```

#### When to Use Services vs Topics

- **Use Services when**:
  - You need request/response interaction
  - Operations have a clear beginning and end
  - You need guaranteed delivery and response
  - The operation is relatively fast (under a few seconds)
  - You need to pass parameters and get computed results

- **Use Topics when**:
  - You need continuous data streaming
  - Communication should be decoupled in time
  - Multiple subscribers need the same data
  - Data is updated continuously (sensor data, status updates)

### ROS 2 Packages

Packages are the basic building and distribution units in ROS 2. They contain source code, data, and configuration files organized in a standard structure. Packages provide a way to organize related functionality, manage dependencies, and distribute ROS 2 software.

#### Package Structure

A standard ROS 2 package follows this structure:

```
package_name/
├── CMakeLists.txt         # Build configuration for C++ packages
├── package.xml            # Package metadata and dependencies
├── src/                   # Source code files
│   ├── cpp files          # C++ source code
│   └── python files       # Python source code
├── include/               # Header files (C++)
├── launch/                # Launch files for starting multiple nodes
├── config/                # Configuration files
├── test/                  # Unit and integration tests
├── scripts/               # Standalone executable scripts
├── msg/                   # Custom message definitions
├── srv/                   # Custom service definitions
├── action/                # Custom action definitions
└── setup.py               # Python package setup (for Python packages)
```

#### Package Metadata (package.xml)

The `package.xml` file contains metadata about the package and its dependencies:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_robot_package</name>
  <version>0.1.0</version>
  <description>A package for my robot functionality</description>
  <maintainer email="user@example.com">User Name</maintainer>
  <license>Apache-2.0</license>

  <!-- Build tool dependencies -->
  <buildtool_depend>ament_cmake</buildtool_depend>

  <!-- Runtime dependencies -->
  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>geometry_msgs</depend>

  <!-- Test dependencies -->
  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

#### Creating Packages

Packages can be created using the `ros2 pkg create` command:

```bash
# Create a Python package
ros2 pkg create --build-type ament_python my_python_package

# Create a C++ package
ros2 pkg create --build-type ament_cmake my_cpp_package

# Create a package with dependencies
ros2 pkg create --build-type ament_python my_package \
  --dependencies rclpy std_msgs sensor_msgs geometry_msgs
```

#### Python Package Setup (setup.py)

For Python packages, the `setup.py` file configures the Python build process:

```python
from setuptools import setup
import os
from glob import glob

package_name = 'my_robot_package'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Include config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User Name',
    maintainer_email='user@example.com',
    description='A package for my robot functionality',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_node = my_robot_package.my_node:main',
            'another_node = my_robot_package.another_node:main',
        ],
    },
)
```

#### Package Management Tools

ROS 2 provides several tools for managing packages:

- **ros2 pkg**: Package management commands
  - `ros2 pkg list`: List all available packages
  - `ros2 pkg xml <package_name>`: Show package.xml content
  - `ros2 pkg executables <package_name>`: List executables in a package

- **colcon**: Build system for ROS 2 packages
  - `colcon build`: Build all packages in the workspace
  - `colcon build --packages-select <package_name>`: Build specific package
  - `colcon test`: Run tests for packages

- **ament**: Build system and testing framework
  - Provides build tools, test frameworks, and linters
  - Supports multiple build systems (CMake, Python)

#### Entry Points and Executables

Python packages can define executable entry points in `setup.py`:

```python
entry_points={
    'console_scripts': [
        'my_robot_node = my_robot_package.robot_node:main',
        'my_robot_cli = my_robot_package.cli:main',
    ],
},
```

This allows nodes to be executed directly from the command line:

```bash
# After building the package
ros2 run my_robot_package my_robot_node
```

#### Package Dependencies

Dependencies in ROS 2 packages are categorized as:

- **buildtool_depend**: Build system dependencies (e.g., `ament_cmake`)
- **build_depend**: Dependencies needed during build time
- **exec_depend**: Dependencies needed at runtime
- **test_depend**: Dependencies needed for testing
- **depend**: Shorthand for both build and exec dependencies

#### Best Practices for Package Organization

- Use descriptive package names that clearly indicate functionality
- Follow the ROS 2 naming conventions (lowercase, underscores)
- Keep related functionality together in the same package
- Separate different types of functionality into different packages
- Use appropriate dependency management
- Include proper documentation and examples
- Write comprehensive tests for your packages

## Code Examples

### Creating a Basic ROS 2 Node

Here's a basic ROS 2 Python node that demonstrates the fundamental structure:

```python
import rclpy
from rclpy.node import Node

class BasicNode(Node):
    def __init__(self):
        super().__init__('basic_node')
        self.get_logger().info('Basic Node initialized')

def main(args=None):
    rclpy.init(args=args)

    basic_node = BasicNode()

    try:
        rclpy.spin(basic_node)
    except KeyboardInterrupt:
        pass
    finally:
        basic_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Publisher Node Example

Here's a ROS 2 node that publishes messages to a topic:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')
        self.publisher = self.create_publisher(String, 'topic_name', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.get_logger().info('Publisher Node initialized')

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)

    publisher_node = PublisherNode()

    try:
        rclpy.spin(publisher_node)
    except KeyboardInterrupt:
        pass
    finally:
        publisher_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Subscriber Node Example

Here's a ROS 2 node that subscribes to messages from a topic:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber_node')
        self.subscription = self.create_subscription(
            String,
            'topic_name',
            self.listener_callback,
            10)
        self.subscription  # Prevent unused variable warning
        self.get_logger().info('Subscriber Node initialized')

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)

    subscriber_node = SubscriberNode()

    try:
        rclpy.spin(subscriber_node)
    except KeyboardInterrupt:
        pass
    finally:
        subscriber_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Service Server Example

Here's a ROS 2 node that implements a service server:

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class ServiceServerNode(Node):
    def __init__(self):
        super().__init__('service_server_node')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
        self.get_logger().info('Service Server Node initialized')

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Returning {request.a} + {request.b} = {response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)

    service_server_node = ServiceServerNode()

    try:
        rclpy.spin(service_server_node)
    except KeyboardInterrupt:
        pass
    finally:
        service_server_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Service Client Example

Here's a ROS 2 node that acts as a service client:

```python
import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class ServiceClientNode(Node):
    def __init__(self):
        super().__init__('service_client_node')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = AddTwoInts.Request()
        self.get_logger().info('Service Client Node initialized')

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)

    service_client_node = ServiceClientNode()

    try:
        # Send request with values from command line arguments or default
        a = int(sys.argv[1]) if len(sys.argv) > 1 else 1
        b = int(sys.argv[2]) if len(sys.argv) > 2 else 2

        response = service_client_node.send_request(a, b)
        service_client_node.get_logger().info(f'Result of {a} + {b} = {response.sum}')
    except Exception as e:
        service_client_node.get_logger().error(f'Service call failed: {e}')
    finally:
        service_client_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Package Structure Example

Here's how to create a basic ROS 2 package structure:

```bash
# Create a new ROS 2 package
ros2 pkg create --build-type ament_python my_robot_package

# Or for C++ package
ros2 pkg create --build-type ament_cmake my_robot_package
```

The package.xml file would look like:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_robot_package</name>
  <version>0.0.0</version>
  <description>ROS 2 package for robot functionality</description>
  <maintainer email="user@example.com">User Name</maintainer>
  <license>Apache-2.0</license>

  <exec_depend>rclpy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

### Parameters Example

Here's how to use parameters in a ROS 2 node:

```python
import rclpy
from rclpy.node import Node

class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')

        # Declare parameters with default values
        self.declare_parameter('robot_name', 'turtlebot')
        self.declare_parameter('max_velocity', 1.0)
        self.declare_parameter('use_sim_time', False)

        # Get parameter values
        robot_name = self.get_parameter('robot_name').value
        max_velocity = self.get_parameter('max_velocity').value
        use_sim_time = self.get_parameter('use_sim_time').value

        self.get_logger().info(f'Robot name: {robot_name}')
        self.get_logger().info(f'Max velocity: {max_velocity}')
        self.get_logger().info(f'Use sim time: {use_sim_time}')

        # Set a parameter callback to handle parameter changes
        self.add_on_set_parameters_callback(self.parameter_callback)

    def parameter_callback(self, params):
        for param in params:
            self.get_logger().info(f'Parameter {param.name} changed to {param.value}')
        return rclpy.node.SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)

    parameter_node = ParameterNode()

    try:
        rclpy.spin(parameter_node)
    except KeyboardInterrupt:
        pass
    finally:
        parameter_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Timer Example

Here's how to use timers in a ROS 2 node:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TimerNode(Node):
    def __init__(self):
        super().__init__('timer_node')

        # Create a publisher
        self.publisher = self.create_publisher(String, 'timer_topic', 10)

        # Create a timer that runs every 0.5 seconds
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Counter for messages
        self.i = 0

        self.get_logger().info('Timer Node initialized')

    def timer_callback(self):
        msg = String()
        msg.data = f'Timer message: {self.i}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)

    timer_node = TimerNode()

    try:
        rclpy.spin(timer_node)
    except KeyboardInterrupt:
        pass
    finally:
        timer_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Exercises

1. Create a simple publisher and subscriber pair that communicate temperature data
2. Implement a service that converts coordinates from one frame to another
3. Create a package that includes both publisher and subscriber nodes
4. Experiment with different QoS (Quality of Service) settings for topics

## References

- ROS 2 Documentation: https://docs.ros.org/en/humble/
- ROS 2 Tutorials: https://docs.ros.org/en/humble/Tutorials.html
- Designing and Building ROS 2 Packages: https://docs.ros.org/en/humble/How-To-Guides/Creating-Your-First-ROS2-Package.html
- ROS 2 Concepts: https://docs.ros.org/en/humble/Concepts/About-ROS-2-Concepts.html