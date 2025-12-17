# ROS 2 Basics: Nodes, Topics, and Services

ROS 2 (Robot Operating System 2) is a powerful framework that enables various components of a robotic system to communicate and work together. At its core, ROS 2 relies on a few fundamental concepts: Nodes, Topics, and Services.

## Nodes

A **Node** is an executable process in ROS 2. It's an individual computational unit that performs a specific task. For example, one node might control a motor, another might process camera data, and a third might handle navigation logic.

*   **Characteristics**:
    *   Each node should ideally be responsible for a single, well-defined task.
    *   Nodes can be written in different programming languages (e.g., Python, C++).
    *   Multiple instances of the same node can run simultaneously, each with a unique name.

### Step-by-Step: Creating a Node

1. **Node Architecture**: A node contains the logic for a specific robot function. It can communicate with other nodes through topics, services, and actions.

2. **Node Lifecycle**:
   - Initialization: The node connects to the ROS 2 graph
   - Execution: The node performs its function (publishing, subscribing, serving)
   - Shutdown: The node cleanly disconnects from the ROS 2 graph

3. **Node Naming**: Each node must have a unique name within the ROS 2 system to prevent conflicts.

4. **Node Parameters**: Nodes can accept parameters at runtime to modify their behavior without recompiling.

## Topics

**Topics** are the primary mechanism for asynchronous, one-way data streaming in ROS 2. Nodes publish data to topics, and other nodes subscribe to those topics to receive the data. This is a many-to-many communication model.

*   **Key Concepts**:
    *   **Publisher**: A node that sends messages to a topic.
    *   **Subscriber**: A node that receives messages from a topic.
    *   **Message Type**: Every topic has an associated message type that defines the structure of the data being transmitted (e.g., `sensor_msgs/msg/LaserScan`, `std_msgs/msg/String`).

*   **Example Use Cases**:
    *   A camera node publishes image data to an `/image` topic.
    *   A LiDAR node publishes laser scan data to a `/scan` topic.
    *   A control node publishes motor commands to a `/cmd_vel` topic.

### Step-by-Step: Working with Topics

1. **Topic Creation**: A publisher node creates a topic and begins publishing messages to it.

2. **Message Structure**: Each topic has a specific message type that defines the data structure. Common types include:
   - `std_msgs/String`: Simple text messages
   - `sensor_msgs/Image`: Camera image data
   - `sensor_msgs/LaserScan`: LiDAR data
   - `geometry_msgs/Twist`: Velocity commands for robot motion

3. **QoS (Quality of Service)**: Topics can have different QoS profiles that define how messages are handled:
   - Reliability: Reliable vs. best-effort delivery
   - Durability: Keep last message vs. volatile
   - History: How many messages to keep in the queue

4. **Topic Discovery**: When a node subscribes to a topic, ROS 2 automatically connects them if the topic exists.

5. **Data Flow**: Topics enable one-to-many or many-to-many communication patterns where multiple publishers can send to the same topic and multiple subscribers can receive from it.

## Services

**Services** are used for synchronous, request-response communication between nodes. When a node needs to perform an action or retrieve information from another node and expects an immediate response, it uses a service. This is a one-to-one communication model.

*   **Key Concepts**:
    *   **Service Server**: A node that provides a service and waits for requests.
    *   **Service Client**: A node that sends a request to a service server and waits for a response.
    *   **Service Type**: Defines the structure of both the request and response messages (e.g., `std_srvs/srv/Trigger`).

*   **Example Use Cases**:
    *   A client requests a robot to "go home" from a navigation service.
    *   A client requests the current battery status from a power management service.
    *   A client triggers a specific action on a robotic arm.

### Step-by-Step: Working with Services

1. **Service Definition**: Services are defined by a service file that specifies the request and response message types.

2. **Service Server**: A node implements a service server that waits for incoming requests and processes them.

3. **Service Client**: A node acts as a service client by sending requests and waiting for responses.

4. **Synchronous Communication**: The client waits for the server to process the request and return a response before continuing.

5. **Service Types**: Common service types include:
   - `std_srvs/Empty`: Simple service with no request or response
   - `std_srvs/Trigger`: Service that returns success status
   - `std_srvs/SetBool`: Service to set a boolean value

## Actions

**Actions** are used for goal-oriented communication that may take a long time to complete. They provide feedback during execution and can be canceled.

*   **Key Concepts**:
    *   **Action Server**: A node that executes long-running goals.
    *   **Action Client**: A node that sends goals to an action server.
    *   **Action Type**: Defines goal, feedback, and result message structures.

### Step-by-Step: Working with Actions

1. **Goal Request**: An action client sends a goal to an action server.

2. **Feedback**: The server sends periodic feedback updates during execution.

3. **Result**: The server sends a final result when the goal is completed (or failed/canceled).

4. **Cancelation**: The client can cancel a goal at any time.

5. **Use Cases**: Actions are ideal for navigation goals, trajectory execution, or any task that takes time and needs monitoring.

## ROS 2 Architecture

### DDS (Data Distribution Service)

ROS 2 uses DDS as its underlying middleware for communication. DDS provides:
- Discovery: Nodes automatically find each other
- Transport: Message delivery between nodes
- Quality of Service: Configurable reliability and performance settings

### The ROS 2 Graph

The ROS 2 graph represents all nodes, topics, services, and actions in the system. Tools like `rqt_graph` can visualize this graph.

## ROS 2 CLI Tools

ROS 2 provides command-line tools to inspect and interact with nodes, topics, and services.

*   `ros2 node list`: List active nodes.
*   `ros2 node info <node_name>`: Get detailed information about a specific node.
*   `ros2 topic list`: List active topics.
*   `ros2 topic echo <topic_name>`: Display messages published on a topic.
*   `ros2 topic info <topic_name>`: Get information about a topic (publishers/subscribers).
*   `ros2 service list`: List available services.
*   `ros2 service call <service_name> <service_type> <arguments>`: Call a service.
*   `ros2 action list`: List available actions.
*   `ros2 run <package_name> <executable_name>`: Run a node directly.
*   `ros2 launch <package_name> <launch_file>`: Launch multiple nodes at once.

## Advanced Concepts

### Namespaces

Nodes and topics can be organized using namespaces to avoid naming conflicts:
- `/robot1/joint_states` and `/robot2/joint_states` for multiple robots
- Namespaces help organize complex multi-robot systems

### Parameters

Nodes can have configurable parameters that can be set at launch time:
- `ros2 param list`: List parameters for a node
- `ros2 param get <node_name> <param_name>`: Get a parameter value
- `ros2 param set <node_name> <param_name> <value>`: Set a parameter value

### Lifecycle Nodes

For complex systems, ROS 2 provides lifecycle nodes that have explicit state management:
- Unconfigured → Inactive → Active → Finalized
- Provides better control over node initialization and shutdown

Understanding these basic communication primitives is essential for building complex robotic applications with ROS 2. The next section will explore how to implement these concepts using Python.
