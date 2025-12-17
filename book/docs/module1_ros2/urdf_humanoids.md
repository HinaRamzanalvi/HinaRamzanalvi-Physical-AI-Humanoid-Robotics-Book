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

### Step-by-Step: Understanding URDF Elements

1. **Robot Element**: The root element that contains the entire robot description and has a name attribute.

2. **Link Elements**: Each rigid body part of the robot, containing:
   - Visual: How the link appears in simulation and visualization
   - Collision: How the link interacts with physics simulation
   - Inertial: Physical properties for dynamics simulation

3. **Joint Elements**: Connections between links, defining how they can move relative to each other.

4. **Materials**: Define colors and textures for visual elements.

## 2. Detailed URDF Components

### Link Elements

Each link can contain several sub-elements:

```xml
<link name="link_name">
  <!-- Inertial properties for physics simulation -->
  <inertial>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <mass value="1.0"/>
    <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
  </inertial>

  <!-- Visual properties for rendering -->
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="1 1 1"/>
    </geometry>
    <material name="red">
      <color rgba="1 0 0 1"/>
    </material>
  </visual>

  <!-- Collision properties for physics -->
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="1 1 1"/>
    </geometry>
  </collision>
</link>
```

### Joint Types

*   **`fixed`**: No movement between parent and child links
*   **`revolute`**: Rotational joint with limited range
*   **`continuous`**: Rotational joint with unlimited range
*   **`prismatic`**: Linear sliding joint with limited range
*   **`floating`**: 6 degrees of freedom (rarely used)
*   **`planar`**: Movement on a plane (rarely used)

### Geometry Types

*   **`box`**: Rectangular box with size="x y z"
*   **`cylinder`**: Cylinder with radius and length
*   **`sphere`**: Sphere with radius
*   **`mesh`**: Complex shape loaded from external file (STL, DAE, etc.)

## 3. Example: Simple Robotic Arm URDF

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

## 4. Step-by-Step: Creating a Humanoid URDF

### Step 1: Define the Robot Root

```xml
<?xml version="1.0"?>
<robot name="humanoid_robot">
  <!-- All links and joints will go here -->
</robot>
```

### Step 2: Create the Torso

The torso is typically the root link of a humanoid robot:

```xml
  <!-- Torso (root link) -->
  <link name="torso">
    <visual>
      <origin xyz="0 0 0.5" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.2 1.0"/>
      </geometry>
      <material name="light_gray">
        <color rgba="0.7 0.7 0.7 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.5" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.2 1.0"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <origin xyz="0 0 0.5" rpy="0 0 0"/>
      <inertia ixx="0.5" ixy="0.0" ixz="0.0" iyy="0.5" iyz="0.0" izz="0.2"/>
    </inertial>
  </link>
```

### Step 3: Add the Head

```xml
  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="skin">
        <color rgba="0.8 0.6 0.4 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Neck joint -->
  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 1.0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="5" velocity="2"/>
  </joint>
```

### Step 4: Add Arms

```xml
  <!-- Left shoulder -->
  <link name="left_shoulder">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.05"/>
      </geometry>
      <material name="joint_color">
        <color rgba="0.5 0.5 0.5 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.1" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.0005"/>
    </inertial>
  </link>

  <joint name="left_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_shoulder"/>
    <origin xyz="0.15 0 0.7" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="2"/>
  </joint>

  <!-- Left upper arm -->
  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
      <material name="arm_color">
        <color rgba="0.6 0.6 0.8 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="left_elbow_joint" type="revolute">
    <parent link="left_shoulder"/>
    <child link="left_upper_arm"/>
    <origin xyz="0 0 -0.05" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-2.0" upper="0.5" effort="10" velocity="2"/>
  </joint>

  <!-- Add more joints and links for lower arms and hands -->
```

### Step 5: Add Legs

```xml
  <!-- Left hip -->
  <link name="left_hip">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.06"/>
      </geometry>
      <material name="joint_color">
        <color rgba="0.5 0.5 0.5 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.1" radius="0.06"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.8"/>
      <inertia ixx="0.002" ixy="0.0" ixz="0.0" iyy="0.002" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="left_hip_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_hip"/>
    <origin xyz="-0.05 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-0.5" upper="0.5" effort="15" velocity="2"/>
  </joint>

  <!-- Left thigh -->
  <link name="left_thigh">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
      <material name="leg_color">
        <color rgba="0.4 0.4 0.6 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.03" ixy="0.0" ixz="0.0" iyy="0.03" iyz="0.0" izz="0.002"/>
    </inertial>
  </link>

  <joint name="left_knee_joint" type="revolute">
    <parent link="left_hip"/>
    <child link="left_thigh"/>
    <origin xyz="0 0 -0.05" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="2.0" effort="15" velocity="2"/>
  </joint>

  <!-- Add more joints and links for lower legs and feet -->
```

## 5. Advanced URDF Features

### Using Xacro for Complex Robots

Xacro (XML Macros) allows you to create more maintainable URDF files:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid_with_xacro">

  <!-- Define constants -->
  <xacro:property name="M_PI" value="3.1415926535897931" />
  <xacro:property name="torso_height" value="1.0" />
  <xacro:property name="torso_width" value="0.3" />
  <xacro:property name="torso_depth" value="0.2" />

  <!-- Macro for creating an arm -->
  <xacro:macro name="arm" params="side reflect">
    <link name="${side}_shoulder">
      <visual>
        <geometry>
          <cylinder length="0.1" radius="0.05"/>
        </geometry>
      </visual>
      <collision>
        <geometry>
          <cylinder length="0.1" radius="0.05"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="0.5"/>
        <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.0005"/>
      </inertial>
    </link>

    <joint name="${side}_shoulder_joint" type="revolute">
      <parent link="torso"/>
      <child link="${side}_shoulder"/>
      <origin xyz="${reflect * torso_width/2} 0 0.7" rpy="0 0 0"/>
      <axis xyz="0 0 1"/>
      <limit lower="-1.57" upper="1.57" effort="10" velocity="2"/>
    </joint>
  </xacro:macro>

  <!-- Use the macro to create both arms -->
  <xacro:arm side="left" reflect="1"/>
  <xacro:arm side="right" reflect="-1"/>
</robot>
```

### Gazebo-Specific Elements

For simulation in Gazebo, you can add Gazebo-specific elements:

```xml
  <gazebo reference="torso">
    <material>Gazebo/Gray</material>
    <mu1>0.2</mu1>
    <mu2>0.2</mu2>
  </gazebo>

  <gazebo>
    <plugin name="imu_plugin" filename="libgazebo_ros_imu.so">
      <topicName>imu/data</topicName>
      <bodyName>torso</bodyName>
    </plugin>
  </gazebo>
```

## 6. Visualizing URDF

### Using RViz

You can visualize URDF files using `rviz` (a 3D visualizer in ROS).

1.  **Launch `rviz`**:
    ```bash
    rviz2
    ```
2.  **Add RobotModel**: In the Rviz display panel, click "Add" and select "RobotModel".
3.  **Set `robot_description`**: If your URDF is loaded via a ROS parameter, `rviz` will automatically pick it up. Otherwise, you might need to load it manually or use a launch file.

### Using Robot State Publisher

To properly visualize a URDF in RViz, you typically need to run the robot state publisher:

```bash
# First, load the URDF to the parameter server
ros2 param set /robot_state_publisher robot_description --string-type "$(cat path/to/your/robot.urdf)"

# Then run the robot state publisher
ros2 run robot_state_publisher robot_state_publisher
```

### Using Joint State Publisher GUI

To interactively control joint positions:

```bash
ros2 run joint_state_publisher_gui joint_state_publisher_gui
```

## 7. URDF for Humanoids: Special Considerations

### Kinematic Chains

Designing URDF for humanoids is more complex due to the higher number of degrees of freedom (DOF) and intricate link structures. Key considerations include:

*   **Kinematic Chains**: Defining hierarchical parent-child relationships for limbs (e.g., torso -> shoulder -> upper arm -> forearm -> hand).
*   **Joint Limits**: Accurately specifying the range of motion for each joint to mimic human-like movements and prevent self-collisions.
*   **Collision Models**: Creating simplified collision geometries for efficient physics simulation while maintaining realism.
*   **Mass Distribution**: Properly distributing mass across links to ensure realistic dynamics.

### Balance and Stability

Humanoid robots require special attention to:

*   **Center of Mass**: Ensuring the overall center of mass stays within the support polygon during movement
*   **Zero Moment Point (ZMP)**: Critical for stable walking patterns
*   **Foot Contact**: Proper collision geometry for feet to interact with the ground

### Humanoid-Specific Joints

*   **Ball Joints**: For shoulders and hips to allow multi-axis movement
*   **Revolute Joints**: For elbows, knees, wrists, and ankles
*   **Fixed Joints**: For connecting sensors or cosmetic elements

## 8. Best Practices for Humanoid URDF

1. **Start Simple**: Begin with a basic skeleton and gradually add complexity
2. **Use Realistic Dimensions**: Base link sizes on actual human proportions
3. **Proper Inertial Properties**: Accurate mass and inertia values for realistic simulation
4. **Collision Optimization**: Use simple shapes for collision while maintaining visual detail
5. **Xacro for Complex Models**: Use macros to avoid repetition and improve maintainability
6. **Test in Simulation**: Validate the URDF in Gazebo before moving to hardware
7. **Documentation**: Comment your URDF files to explain the purpose of each link and joint

Tools like Blender or CAD software are often used to design the visual and collision meshes, which are then referenced in the URDF. Understanding URDF is fundamental for bringing your humanoid robot designs to life in simulation and eventually controlling them in the physical world.
