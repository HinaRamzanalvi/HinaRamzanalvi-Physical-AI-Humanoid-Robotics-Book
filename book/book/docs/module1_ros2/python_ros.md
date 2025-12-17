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

## 2. Creating a Simple Publisher Node

A publisher node sends data to a ROS 2 topic.

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

You should see the subscriber printing the messages published by the publisher. This demonstrates the fundamental topic-based communication in ROS 2 using `rclpy`.
