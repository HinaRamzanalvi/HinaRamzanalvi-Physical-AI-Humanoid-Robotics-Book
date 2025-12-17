# Python for ROS 2: rclpy

ROS 2 provides client libraries for different programming languages, allowing developers to write nodes in their preferred language. For Python, the client library is `rclpy`. This section will guide you through creating ROS 2 nodes, publishers, and subscribers using `rclpy`.

## 1. Setting up a ROS 2 Python Package

Before writing Python nodes, you typically set up a ROS 2 Python package.

### Creating a Package

In your ROS 2 workspace (e.g., `~/ros_ws/src`), you can create a new Python package using `ros2 pkg create`:

```bash
ros2 pkg create --build-type ament_python my_python_pkg
```

This creates a directory `my_python_pkg` with a basic structure, including a `setup.py` and `package.xml` for package management.

### Step-by-Step: Package Structure

1. **Package Directory**: The command creates a directory with the following structure:
   ```
   my_python_pkg/
   ├── my_python_pkg/
   │   ├── __init__.py
   │   └── my_python_pkg
   ├── package.xml
   ├── setup.cfg
   ├── setup.py
   └── test/
       ├── __init__.py
       ├── test_copyright.py
       ├── test_flake8.py
       └── test_pep257.py
   ```

2. **package.xml**: Contains package metadata including name, version, description, maintainers, licenses, and dependencies.

3. **setup.py**: Python setup script that defines how the package is built and installed.

4. **setup.cfg**: Configuration file for installation paths.

5. **Source Directory**: The inner `my_python_pkg` directory is where your Python nodes will go.

## 2. Creating a Simple Publisher Node

A publisher node sends data to a ROS 2 topic.

### Step-by-Step: Creating a Publisher

1. **Import Required Modules**:
   - `rclpy`: The ROS 2 Python client library
   - `Node`: The base class for ROS 2 nodes
   - Message types (e.g., `std_msgs.msg.String`): Defines the structure of messages

2. **Create Node Class**:
   - Inherit from `Node`
   - Call `super().__init__()` with a unique node name
   - Create publisher using `self.create_publisher()`
   - Set up a timer for periodic publishing

3. **Implement Publishing Logic**:
   - Create message instances
   - Fill message data
   - Publish using the publisher object
   - Log messages for debugging

### Example: `simple_publisher.py`

Create a file `my_python_pkg/my_python_pkg/simple_publisher.py`:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.get_logger().info('Simple Publisher Node has been started.')

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello ROS 2 from Python: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    simple_publisher = SimplePublisher()
    rclpy.spin(simple_publisher)
    simple_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Modifying `setup.py`

To make your Python node executable, you need to add an entry point in `my_python_pkg/setup.py`:

```python
# ... (existing imports)

setup(
    # ... (existing metadata)
    entry_points={
        'console_scripts': [
            'simple_publisher = my_python_pkg.simple_publisher:main',
        ],
    },
)
```

### Building and Running

1.  **Build**: In your `ros_ws` directory:
    ```bash
    colcon build --packages-select my_python_pkg
    ```
2.  **Source**:
    ```bash
    source install/setup.bash
    ```
3.  **Run**:
    ```bash
    ros2 run my_python_pkg simple_publisher
    ```

## 3. Creating a Simple Subscriber Node

A subscriber node listens for data on a ROS 2 topic.

### Step-by-Step: Creating a Subscriber

1. **Import Required Modules**: Same as publisher but focused on subscription.

2. **Create Node Class**:
   - Inherit from `Node`
   - Call `super().__init__()` with a unique node name
   - Create subscription using `self.create_subscription()`
   - Define callback function to handle incoming messages

3. **Implement Message Handling**:
   - Define callback function that receives messages
   - Process message data
   - Log messages for debugging

### Example: `simple_subscriber.py`

Create a file `my_python_pkg/my_python_pkg/simple_subscriber.py`:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__('simple_subscriber')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info('Simple Subscriber Node has been started.')

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    simple_subscriber = SimpleSubscriber()
    rclpy.spin(simple_subscriber)
    simple_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Modifying `setup.py`

Add another entry point for the subscriber in `my_python_pkg/setup.py`:

```python
# ... (existing imports)

setup(
    # ... (existing metadata)
    entry_points={
        'console_scripts': [
            'simple_publisher = my_python_pkg.simple_publisher:main',
            'simple_subscriber = my_python_pkg.simple_subscriber:main', # New entry
        ],
    },
)
```

### Building and Running

1.  **Build (again)**:
    ```bash
    colcon build --packages-select my_python_pkg
    ```
2.  **Source (again)**:
    ```bash
    source install/setup.bash
    ```
3.  **Run in separate terminals**:
    ```bash
    ros2 run my_python_pkg simple_publisher
    ```
    ```bash
    ros2 run my_python_pkg simple_subscriber
    ```

## 4. Advanced Publisher Features

### Quality of Service (QoS) Settings

You can customize QoS profiles for publishers:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# Create a custom QoS profile
qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)

self.publisher_ = self.create_publisher(String, 'chatter', qos_profile)
```

### Publishing Different Message Types

ROS 2 supports many message types:

```python
from geometry_msgs.msg import Twist  # For robot motion commands
from sensor_msgs.msg import LaserScan, Image  # For sensor data
from std_msgs.msg import Int32, Float64  # For numeric data
```

## 5. Advanced Subscriber Features

### Message Filtering and Processing

```python
def listener_callback(self, msg):
    # Process message data
    processed_data = self.process_message(msg.data)

    # Perform actions based on message content
    if 'urgent' in msg.data:
        self.handle_urgent_message()
```

### Multiple Subscriptions

A node can subscribe to multiple topics:

```python
def __init__(self):
    super().__init__('multi_subscriber')

    self.subscription1 = self.create_subscription(
        String, 'topic1', self.callback1, 10)

    self.subscription2 = self.create_subscription(
        Int32, 'topic2', self.callback2, 10)
```

## 6. Creating a Service Server

### Step-by-Step: Service Implementation

1. **Import Service Types**: Import the service message type you want to use.

2. **Create Service Server**: Use `create_service()` method to create a server.

3. **Implement Service Callback**: Define the function that handles service requests.

### Example: `simple_service_server.py`

```python
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool

class SimpleServiceServer(Node):
    def __init__(self):
        super().__init__('simple_service_server')
        self.srv = self.create_service(
            SetBool,
            'set_boolean',
            self.set_boolean_callback
        )
        self.get_logger().info('Service server started')

    def set_boolean_callback(self, request, response):
        response.success = True
        response.message = f'Received request: {request.data}'
        self.get_logger().info(f'Returning: {response.success} with message: {response.message}')
        return response

def main(args=None):
    rclpy.init(args=args)
    simple_service_server = SimpleServiceServer()
    rclpy.spin(simple_service_server)
    simple_service_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 7. Creating a Service Client

### Example: `simple_service_client.py`

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_srvs.srv import SetBool

class SimpleServiceClient(Node):
    def __init__(self):
        super().__init__('simple_service_client')
        self.cli = self.create_client(SetBool, 'set_boolean')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = SetBool.Request()

    def send_request(self, data):
        self.req.data = data
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    simple_service_client = SimpleServiceClient()
    response = simple_service_client.send_request(True)
    simple_service_client.get_logger().info(
        f'Result: {response.success}, Message: {response.message}'
    )
    simple_service_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 8. Working with Parameters

### Setting and Getting Parameters

```python
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')

        # Declare parameters with default values
        self.declare_parameter('my_parameter', 'default_value')
        self.declare_parameter('threshold', 10.0)

        # Get parameter values
        self.param_value = self.get_parameter('my_parameter').value
        self.threshold = self.get_parameter('threshold').value

        self.get_logger().info(f'Parameter value: {self.param_value}')
        self.get_logger().info(f'Threshold: {self.threshold}')

        # Add callback for parameter changes
        self.add_on_set_parameters_callback(self.parameter_callback)

    def parameter_callback(self, params):
        for param in params:
            self.get_logger().info(f'Parameter {param.name} changed to {param.value}')
        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)
    parameter_node = ParameterNode()
    rclpy.spin(parameter_node)
    parameter_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 9. Error Handling and Best Practices

### Error Handling in ROS 2 Nodes

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import traceback

class RobustNode(Node):
    def __init__(self):
        super().__init__('robust_node')
        self.publisher_ = self.create_publisher(String, 'robust_topic', 10)

        # Add try-catch blocks for critical operations
        try:
            self.timer = self.create_timer(0.5, self.safe_timer_callback)
        except Exception as e:
            self.get_logger().error(f'Failed to create timer: {e}')

    def safe_timer_callback(self):
        try:
            msg = String()
            msg.data = 'Safe message'
            self.publisher_.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Error in timer callback: {e}')
            traceback.print_exc()
```

### Best Practices

1. **Node Lifecycle**: Always properly initialize and clean up resources
2. **Logging**: Use appropriate log levels (info, warn, error, debug)
3. **Error Handling**: Implement proper exception handling
4. **Resource Management**: Properly destroy nodes and clean up resources
5. **Naming Conventions**: Use consistent and descriptive names
6. **Documentation**: Document your code and nodes properly

You should see the subscriber printing the messages published by the publisher. This demonstrates the fundamental topic-based communication in ROS 2 using `rclpy`.
