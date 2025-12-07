# Digital Twin Workstation Setup Guide

This guide provides instructions for setting up your Digital Twin Workstation with Ubuntu 22.04 and NVIDIA GPU drivers, essential for running simulations with Gazebo, Unity, and NVIDIA Isaac Sim.

## 1. Operating System Installation: Ubuntu 22.04 LTS

It is highly recommended to perform a clean installation of Ubuntu 22.04 LTS.

*   **Download**: Obtain the Ubuntu 22.04 LTS ISO from the [official Ubuntu website](https://ubuntu.com/download/desktop).
*   **Installation Media**: Create a bootable USB drive using tools like [Rufus](https://rufus.ie/) (Windows) or [BalenaEtcher](https://www.balena.io/etcher/) (cross-platform).
*   **Installation Process**: Follow the on-screen instructions for a standard Ubuntu installation. Ensure you have at least 200 GB of free disk space.

## 2. NVIDIA GPU Driver Installation

Installing the correct NVIDIA GPU drivers is crucial for hardware acceleration in simulations.

### Option A: Using Ubuntu's Additional Drivers (Recommended)

1.  **Open Software & Updates**: Search for "Software & Updates" in the Ubuntu applications menu.
2.  **Navigate to Additional Drivers**: Go to the "Additional Drivers" tab.
3.  **Select Proprietary Driver**: Choose the latest proprietary NVIDIA driver (e.g., `nvidia-driver-535`, `nvidia-driver-525`). Avoid open-source drivers for optimal performance with NVIDIA tools.
4.  **Apply Changes**: Click "Apply Changes" and follow any prompts.
5.  **Reboot**: Restart your system for the changes to take effect.

### Option B: Manual Installation (Advanced)

Only use this if Option A fails. This method can be more complex and prone to issues.

1.  **Download Driver**: Visit the [NVIDIA driver download page](https://www.nvidia.com/drivers) and select your specific GPU model and operating system.
2.  **Disable Nouveau Driver**:
    *   Create a file `/etc/modprobe.d/blacklist-nouveau.conf` with the following content:
        ```
        blacklist nouveau
        options nouveau modeset=0
        ```
    *   Update the kernel initramfs: `sudo update-initramfs -u`
    *   Reboot: `sudo reboot`
3.  **Run Installer**:
    *   Stop the display server: `sudo systemctl isolate multi-user.target` (or `sudo service lightdm stop`, `sudo service gdm3 stop` depending on your desktop environment).
    *   Navigate to the downloaded driver file (e.g., `NVIDIA-Linux-x86_64-XXX.XX.run`).
    *   Run the installer: `sudo sh NVIDIA-Linux-x86_64-XXX.XX.run`
    *   Follow the prompts.
4.  **Reboot**: `sudo reboot`

## 3. Verify Installation

After rebooting, open a terminal and run:

```bash
nvidia-smi
```

You should see output detailing your NVIDIA GPU and driver version. If not, troubleshoot the driver installation.

## 4. Install Docker (Optional, Recommended for Environment Management)

While not strictly part of the Digital Twin Workstation setup, Docker is highly recommended for managing development environments, especially for ROS 2 and NVIDIA Isaac. Refer to the [Docker Installation Guide for Ubuntu](https://docs.docker.com/engine/install/ubuntu/) for detailed instructions.

## 5. Next Steps

Once your Digital Twin Workstation is set up, you can proceed with setting up specific development environments using the provided Dockerfiles for ROS 2 and NVIDIA Isaac Sim/ROS.
