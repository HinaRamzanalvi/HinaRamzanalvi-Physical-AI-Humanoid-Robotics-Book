## Physics Simulation (Gravity, Collisions, Joints)

In digital twin environments like Gazebo and Unity, accurate physics simulation is paramount for realistic robot behavior. This section covers fundamental physics concepts and how they are implemented in robotics simulators.

### Gravity

Gravity is a constant force that pulls objects towards the center of the Earth. In simulations, it's typically a vector (e.g., `[0, 0, -9.81]` m/s² for Earth's gravity along the Z-axis). Simulators allow you to configure gravity, enabling realistic falling, jumping, and interaction with the ground.

### Collisions

Collision detection and response are crucial for preventing objects from passing through each other and for simulating physical contact.

*   **Collision Shapes:** Simplified geometric primitives (spheres, boxes, cylinders) or convex hulls are used to represent the physical bounds of objects for efficient collision detection. These are often distinct from the visual mesh.
*   **Collision Detection:** Algorithms detect when two collision shapes overlap.
*   **Collision Response:** When a collision is detected, the simulator calculates forces and impulses to prevent interpenetration and simulate realistic bouncing or resting contact, considering properties like friction and restitution.

### Joints

Joints define how different parts of a robot (links) are connected and how they can move relative to each other. Common joint types include:

*   **Revolute Joint:** Allows rotation around a single axis (e.g., an elbow or knee joint).
*   **Prismatic Joint:** Allows linear translation along a single axis (e.g., a linear actuator).
*   **Fixed Joint:** Prevents any relative motion between two links.

Joints have properties like limits (maximum/minimum angle or position), damping, and friction, which contribute to realistic robot kinematics and dynamics.

### Exercises and Tasks

1.  **Conceptual Task:** Describe the difference between a robot's visual mesh and its collision mesh, and explain why both are important in a simulation.
2.  **Simulation Configuration (Conceptual):** If you were setting up a Gazebo simulation for a simple robotic arm, list at least three physics properties you would configure in the world file (e.g., gravity, friction values).
