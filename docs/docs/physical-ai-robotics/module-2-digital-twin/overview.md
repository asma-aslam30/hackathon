---
sidebar_position: 1
sidebar_label: Overview
---

# Module 2: The Digital Twin (Gazebo & Unity) - Overview

## High-level overview
Explore the importance of high-fidelity robotic simulations for development, testing, and training AI agents without real-world risks. Introduce Gazebo and Unity as leading platforms for creating digital twins.

## Deep technical explanation
*   **Gazebo:**
    *   Physics engine (ODE, Bullet, DART, Simbody).
    *   SDF (Simulation Description Format) for robot models and environments.
    *   ROS 2 integration (Gazebo ROS 2 packages).
    *   Sensors in simulation: cameras, LiDAR, IMU, contact sensors.
    *   Creating custom worlds and robots.
*   **Unity:**
    *   Unity Robotics Hub & `ROS-TCP-Connector`.
    *   Creating realistic environments and robot models (URDF import, assets).
    *   Physics engine (NVIDIA PhysX).
    *   ML-Agents integration for reinforcement learning.
    *   High-fidelity rendering and visual effects.

## Real-world examples
Developing Mars rovers, testing drone delivery algorithms, training reinforcement learning agents for complex manipulation tasks.

## Diagrams (Mermaid syntax)
```mermaid
graph TD
    A[Physical Robot] -->|Sensors & Actuators| B(Real World)
    C[Digital Twin (Gazebo/Unity)] -->|Simulated Sensors & Actuators| D(Simulated World)
    B --- F[ROS 2 Control]
    D --- F
    F --> E{AI Brain}
```
*   Comparison of Gazebo vs. Unity features.
*   Data flow from simulated sensors to ROS 2 nodes.

## Code snippet ideas (ROS2 Python & XML/YAML)
*   **Simple SDF model for Gazebo:**
    ```xml
    <model name="simple_box">
      <link name="box_link">
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.01</ixx> <ixy>0.0</ixy> <ixz>0.0</ixz>
            <iyy>0.01</iyy> <iyz>0.0</iyz>
            <izz>0.01</izz>
          </inertia>
        </inertial>
        <visual>
          <geometry><box><size>0.1 0.1 0.1</size></box></geometry>
          <material><script>Gazebo/Green</script></material>
        </visual>
        <collision>
          <geometry><box><size>0.1 0.1 0.1</size></box></geometry>
        </collision>
      </link>
    </model>
    ```
*   **ROS 2 Gazebo plugin example (conceptual `C++` or Python interface):** Attaching a sensor or actuator controller.
*   **Unity C# script for `ROS-TCP-Connector` (conceptual):** Sending and receiving ROS 2 messages.

<h2>Simulation steps (Gazebo, Unity)</h2>
*   **Gazebo:**
    1.  Install Gazebo Fortress/Garden.
    2.  Launch a default empty world.
    3.  Spawn a basic SDF model (e.g., a cube) into the world.
    4.  Integrate `ros_gz_sim` bridge to publish a simulated camera feed to a ROS 2 topic.
    5.  Control a simulated joint using a ROS 2 topic.
*   **Unity:**
    1.  Setup Unity with Robotics Hub.
    2.  Import a simple robot URDF model.
    3.  Configure `ROS-TCP-Connector` to communicate with a local ROS 2 network.
    4.  Create a simple scene with a simulated environment.
    5.  Publish basic robot joint states to ROS 2.

<h2>Hardware & software requirements for this module</h2>
*   **Software:** Ubuntu 22.04+, ROS 2 Humble, Gazebo Fortress/Garden, Unity Hub, Unity 2022.3 LTS+, Unity Robotics Hub, `ROS-TCP-Connector`.
*   **Hardware:** High-performance CPU, NVIDIA GPU (RTX 3060+ recommended for Unity/Isaac Sim), ample RAM (16GB+).

<h2>Mini-tasks for students</h2>
*   Create a custom Gazebo world with obstacles and a simple robotic arm model.
*   Publish joint states from a Unity-simulated robot to a ROS 2 topic.
*   Build a simple perception pipeline: publish a simulated camera image from Gazebo/Unity, subscribe in a ROS 2 node, and apply a basic image filter.

<h2>Learning outcomes</h2>
*   Understand the principles and benefits of robotic simulation.
*   Develop and deploy robot models and environments in Gazebo using SDF.
*   Integrate Gazebo with ROS 2 for sensor data and motor control.
*   Set up a Unity project for robotic simulation and integrate it with ROS 2 using `ROS-TCP-Connector`.
*   Compare and contrast Gazebo and Unity for different simulation needs.

<h2>Integration points for capstone project</h2>
The autonomous humanoid will be extensively developed and tested as a digital twin in both Gazebo (for robust physics) and Unity (for high-fidelity visual perception and advanced RL training).

<h2>Cross-references between modules</h2>
Builds upon ROS 2 communication (Module 1). Provides the simulated environment for AI-Robot Brain (Module 3) and VLA (Module 4).

<h2>Notes for weekly progression (Week 1–13)</h2>
Weeks 4-6: Gazebo and Unity basics, ROS 2 integration.
