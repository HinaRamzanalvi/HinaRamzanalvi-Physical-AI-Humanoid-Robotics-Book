# ROS 2 Basics: Nodes, Topics, and Services

ROS 2 (Robot Operating System 2) is a powerful framework that enables various components of a robotic system to communicate and work together. At its core, ROS 2 relies on a few fundamental concepts: Nodes, Topics, and Services.

## Nodes

A **Node** is an executable process in ROS 2. It's an individual computational unit that performs a specific task. For example, one node might control a motor, another might process camera data, and a third might handle navigation logic.

*   **Characteristics**:
    *   Each node should ideally be responsible for a single, well-defined task.
    *   Nodes can be written in different programming languages (e.g., Python, C++).
    *   Multiple instances of the same node can run simultaneously, each with a unique name.

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

## ROS 2 CLI Tools

ROS 2 provides command-line tools to inspect and interact with nodes, topics, and services.

*   `ros2 node list`: List active nodes.
*   `ros2 topic list`: List active topics.
*   `ros2 topic echo <topic_name>`: Display messages published on a topic.
*   `ros2 service list`: List available services.
*   `ros2 service call <service_name> <service_type> <arguments>`: Call a service.

Understanding these basic communication primitives is essential for building complex robotic applications with ROS 2. The next section will explore how to implement these concepts using Python.
