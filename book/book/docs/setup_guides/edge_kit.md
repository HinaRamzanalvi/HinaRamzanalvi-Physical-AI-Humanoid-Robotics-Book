# Physical AI Edge Kit Setup Guide

This guide provides instructions for setting up your Physical AI Edge Kit, typically an NVIDIA Jetson Orin Nano with connected RealSense camera, USB IMU, and microphone. This setup is crucial for real-world robotics deployments.

## 1. NVIDIA Jetson Orin Nano Setup

### Initial Setup and JetPack Installation

1.  **Flash JetPack OS**: Download the latest [NVIDIA JetPack SDK](https://developer.nvidia.com/embedded/jetpack) for Jetson Orin Nano. Use the SDK Manager to flash the Jetson OS image to your device's storage. This includes Ubuntu, CUDA, cuDNN, TensorRT, and other NVIDIA libraries.
    *   **Refer to**: [NVIDIA Jetson Getting Started Guide](https://developer.nvidia.com/jetson-getting-started)
2.  **Network Configuration**: Connect your Jetson to the internet (Ethernet or Wi-Fi). Configure a static IP address if needed for consistent communication.
3.  **Update System**: After flashing, perform a system update:
    ```bash
    sudo apt update
    sudo apt upgrade -y
    ```

### Install ROS 2 (Humble/Iron)

It's recommended to install ROS 2 on the Jetson directly.

*   **Follow Official ROS 2 Documentation**: Refer to the [official ROS 2 documentation for Ubuntu](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) (choose Humble or Iron, aligning with your workstation). Pay attention to specific instructions for ARM architecture if any.

## 2. RealSense Camera Setup

The RealSense camera is vital for depth perception and RGB imagery.

1.  **Install librealsense SDK**: Follow the [official Intel RealSense SDK installation guide](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md) for Ubuntu.
2.  **Install ROS 2 RealSense Wrapper**: Install the `ros-humble-realsense2-camera` package or build from source if necessary:
    ```bash
    sudo apt install ros-humble-realsense2-camera
    ```
3.  **Verify**: Run `ros2 launch realsense2_camera rs_launch.py` and check for topics like `/camera/depth/image_rect_raw` and `/camera/color/image_raw`.

## 3. USB IMU Setup (Example: BNO055)

A USB IMU provides orientation and acceleration data. This example uses a BNO055.

1.  **Connect IMU**: Plug in your USB IMU.
2.  **Identify Device**: Check `lsusb` and `dmesg` to ensure the system recognizes the IMU.
3.  **Install ROS Driver**: Search for an existing ROS 2 driver (e.g., `ros-humble-bno055`) or a generic `ros-humble-imu-sensor` driver. If not available, building from source might be required.
    *   Example: `sudo apt install ros-humble-imu-bno055` (if available)
4.  **Verify**: Launch the IMU node and check for `/imu/data` topic.

## 4. Microphone Setup

A USB microphone is needed for voice-to-action capabilities.

1.  **Connect Microphone**: Plug in your USB microphone.
2.  **Verify Audio Input**:
    *   Check recognized devices: `arecord -L`
    *   Test recording: `arecord -d 5 test.wav` (records 5 seconds)
3.  **Configure for ROS/Python**: Ensure your Python/ROS applications can access the microphone. This might involve ALSA or PulseAudio configuration depending on your specific setup.

## 5. Next Steps

With your Physical AI Edge Kit configured, you can proceed with deploying and testing ROS 2 applications, AI models from NVIDIA Isaac ROS, and Vision-Language-Action (VLA) pipelines.
