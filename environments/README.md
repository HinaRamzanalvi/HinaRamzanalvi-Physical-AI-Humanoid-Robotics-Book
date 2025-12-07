# Environments for Physical AI & Humanoid Robotics Course

This directory contains Dockerfiles and associated scripts to set up reproducible development environments for the "Physical AI & Humanoid Robotics" course.

## Available Dockerfiles

*   **`Dockerfile.ros2`**: A Dockerfile for a ROS 2 Humble and Python 3.x development environment. This is suitable for general ROS 2 development, package creation, and basic simulation with Gazebo.
*   **`Dockerfile.isaac`**: A Dockerfile for an NVIDIA Isaac Sim/ROS development environment. This extends the ROS 2 environment with NVIDIA CUDA toolkit components, necessary for developing and running Isaac ROS packages.

## Building and Running Docker Containers

### Prerequisites

*   **Docker**: Ensure Docker is installed on your system. Follow the [official Docker installation guide](https://docs.docker.com/engine/install/) for your operating system.
*   **NVIDIA Container Toolkit (for `Dockerfile.isaac`)**: If you intend to use `Dockerfile.isaac` and leverage your NVIDIA GPU, you must install the [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html).

### General Usage

1.  **Navigate to this directory**:
    ```bash
    cd environments
    ```

2.  **Build a Docker image**:
    Replace `<image_name>` with a desired name (e.g., `ros2_dev`, `isaac_dev`) and `<Dockerfile_name>` with the specific Dockerfile (e.g., `Dockerfile.ros2`, `Dockerfile.isaac`).
    ```bash
    docker build -t <image_name> -f <Dockerfile_name> .
    ```

3.  **Run a Docker container**:
    *   **For `Dockerfile.ros2` (ROS 2 Development)**:
        ```bash
        docker run -it --rm --privileged -v /path/to/humanoid_robotics_repo/code_examples:/ros_ws --network host <image_name>
        ```
        -   `--privileged`: Might be needed for certain ROS 2 hardware interactions.
        -   `-v /path/to/humanoid_robotics_repo/code_examples:/ros_ws`: Mounts your host's `code_examples` directory into the container's `/ros_ws`. **Replace `/path/to/humanoid_robotics_repo/code_examples` with the actual absolute path to your `code_examples/` directory.**
        -   `--network host`: Often useful for ROS 2 discovery and communication with host tools.

    *   **For `Dockerfile.isaac` (NVIDIA Isaac Sim/ROS Development)**:
        ```bash
        docker run -it --rm --gpus all -v /path/to/humanoid_robotics_repo/code_examples:/isaac_ros_ws --network host <image_name>
        ```
        -   `--gpus all`: Essential for leveraging your NVIDIA GPU within the container for Isaac ROS.
        -   `-v /path/to/humanoid_robotics_repo/code_examples:/isaac_ros_ws`: Mounts your host's `code_examples` directory into the container's `/isaac_ros_ws`. **Replace `/path/to/humanoid_robotics_repo/code_examples` with the actual absolute path to your `code_examples/` directory.**

## Inside the Container

Once inside the Docker container, you will be in the `/ros_ws` (for ROS 2) or `/isaac_ros_ws` (for Isaac ROS) workspace. You can then:
*   Source the ROS 2 setup: `source /opt/ros/humble/setup.bash`
*   Navigate to specific module code examples: `cd src/module1_ros2/my_robot_pkg/`
*   Build ROS packages: `colcon build`
*   Run ROS nodes: `ros2 run ...`

This setup ensures that all students work within a standardized environment, minimizing "it works on my machine" issues.
