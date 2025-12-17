# Humanoid Robot Description: URDF

The Unified Robot Description Format (URDF) is an XML format used in ROS to describe all aspects of a robot. It allows you to model the robot's kinematics (links and joints), visual properties (geometry, color), and collision properties. For humanoid robots, a URDF file is essential for simulation, visualization, and sometimes even control.

## 1. URDF Structure Basics

A URDF file is composed of `<link>` and `<joint>` elements.

*   **`<link>`**: Represents a rigid body segment of the robot (e.g., torso, upper arm, forearm). Links have inertial, visual, and collision properties.
    *   **`inertial`**: Mass, center of mass, and inertia matrix.
    *   **`visual`**: Geometry (mesh, box, cylinder, sphere), material (color, texture), origin (pose relative to the link's origin).
    *   **`collision`**: Geometry and origin for collision detection. Often simpler than visual geometry for performance.

*   **`<joint>`**: Connects two links, defining their relative motion.
    *   **`parent` and `child`**: Specifies which links the joint connects.
    *   **`type`**: Defines the type of motion (e.g., `revolute` for rotation, `prismatic` for linear, `fixed` for no motion).
    *   **`origin`**: The pose of the child link relative to the parent link.
    *   **`axis`**: The axis of rotation for revolute joints or translation for prismatic joints.
    *   **`limit`**: For revolute/prismatic joints, defines the upper and lower bounds of motion.

## 2. Example: Simple Robotic Arm URDF

Let's consider a very simple two-link robotic arm.

```xml
<?xml version="1.0"?>
<robot name="simple_arm">

  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.05"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.1" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
    </inertial>
  </link>

  <!-- Joint 1: Base to Link 1 -->
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="0.5"/>
  </joint>

  <!-- Link 1 -->
  <link name="link1">
    <visual>
      <geometry>
        <box size="0.1 0.02 0.3"/>
      </geometry>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <material name="green">
        <color rgba="0 0.8 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.1 0.02 0.3"/>
      </geometry>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Joint 2: Link 1 to Link 2 -->
  <joint name="joint2" type="revolute">
    <parent link="link1"/>
    <child link="link2"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="0.5"/>
  </joint>

  <!-- Link 2 -->
  <link name="link2">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.2"/>
      </geometry>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <material name="red">
        <color rgba="0.8 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.05 0.05 0.2"/>
      </geometry>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.0005" ixy="0" ixz="0" iyy="0.0005" iyz="0" izz="0.0001"/>
    </inertial>
  </link>

</robot>
```

## 3. Visualizing URDF

You can visualize URDF files using `rviz` (a 3D visualizer in ROS).

1.  **Launch `rviz`**:
    ```bash
    rviz2
    ```
2.  **Add RobotModel**: In the Rviz display panel, click "Add" and select "RobotModel".
3.  **Set `robot_description`**: If your URDF is loaded via a ROS parameter, `rviz` will automatically pick it up. Otherwise, you might need to load it manually or use a launch file.

## 4. URDF for Humanoids

Designing URDF for humanoids is more complex due to the higher number of degrees of freedom (DOF) and intricate link structures. Key considerations include:

*   **Kinematic Chains**: Defining hierarchical parent-child relationships for limbs (e.g., torso -> shoulder -> upper arm -> forearm -> hand).
*   **Joint Limits**: Accurately specifying the range of motion for each joint to mimic human-like movements and prevent self-collisions.
*   **Collision Models**: Creating simplified collision geometries for efficient physics simulation while maintaining realism.
*   **Meshes**: Using `.stl` or `.dae` mesh files for detailed visual representation of humanoid body parts.

Tools like Blender or CAD software are often used to design the visual and collision meshes, which are then referenced in the URDF. Understanding URDF is fundamental for bringing your humanoid robot designs to life in simulation and eventually controlling them in the physical world.
