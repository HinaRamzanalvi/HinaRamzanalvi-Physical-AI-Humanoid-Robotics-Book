# Code Examples for Physical AI & Humanoid Robotics Course

This directory contains all the code examples, ROS 2 packages, and simulation assets used in the "Physical AI & Humanoid Robotics" course.

## Structure

*   `module1_ros2/`: ROS 2 packages and Python scripts for Module 1.
*   `module2_simulations/`: Gazebo worlds, Unity projects, URDF files for Module 2.
*   `module3_isaac_ai/`: NVIDIA Isaac Sim/ROS projects, AI models for Module 3.
*   `module4_vla/`: VLA integration code, Whisper/LLM examples for Module 4.
*   `capstone_project/`: Integrated capstone project code.

## Using Docker Environments

It is highly recommended to use the provided Docker environments for a consistent and reproducible development experience. The Dockerfiles are located in the `environments/` directory.

### Available Environments

*   **ROS 2 Development**: `environments/Dockerfile.ros2`
    *   Includes ROS 2 Humble, Python 3.x, and common development tools.
    *   **Usage**:
        ```bash
        cd environments
        docker build -t ros2_dev -f Dockerfile.ros2 .
        docker run -it --rm --privileged -v <path_to_code_examples>:/ros_ws --network host ros2_dev
        ```
        Replace `<path_to_code_examples>` with the absolute path to this `code_examples/` directory on your host machine. The `--privileged` flag might be needed for certain ROS 2 hardware interactions. `--network host` is often useful for ROS 2 discovery.

*   **NVIDIA Isaac Sim/ROS Development**: `environments/Dockerfile.isaac`
    *   Extends the ROS 2 environment with NVIDIA CUDA toolkit components, suitable for Isaac ROS development.
    *   **Usage**:
        ```bash
        cd environments
        docker build -t isaac_dev -f Dockerfile.isaac .
        docker run -it --rm --gpus all -v <path_to_code_examples>:/isaac_ros_ws --network host isaac_dev
        ```
        The `--gpus all` flag is essential for leveraging your NVIDIA GPU within the container for Isaac ROS.

## Getting Started with Code Examples

1.  **Clone this repository**: Ensure you have this `humaniod_rebotics` repository cloned to your local machine.
2.  **Navigate to `environments/`**: Change your directory to `environments/`.
3.  **Build Docker images**: Build the necessary Docker images as described above (e.g., `ros2_dev`, `isaac_dev`).
4.  **Run containers**: Start a Docker container for the module you are working on, mounting the `code_examples/` directory.
5.  **Explore Modules**: Navigate into the specific module directories (e.g., `cd /ros_ws/module1_ros2/`) within the container and follow the instructions in their respective `README.md` files or course materials.

## Contributing

If you encounter issues or have improvements, please refer to the main project's contribution guidelines.
