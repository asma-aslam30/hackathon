# Module 2: The Digital Twin (Gazebo & Unity)

This module introduces the concept of Digital Twins in robotics. We will explore how to create, simulate, and interact with virtual replicas of robots and their environments using popular tools like Gazebo and Unity, enabling robust development and testing before physical deployment.

**Learning Outcomes**:

*   Understand the principles and benefits of digital twins in robotics.
*   Set up and utilize Gazebo for robot simulation.
*   Integrate ROS 2 with Gazebo for robot control and data visualization.
*   Explore Unity Robotics Hub for advanced simulation and mixed-reality applications.
*   Compare Gazebo and Unity for different simulation needs.

## 2.1 Introduction to Digital Twins

A Digital Twin is a virtual representation of a physical object, system, or process. In robotics, it allows us to model, simulate, and analyze robot behavior, sensor data, and environmental interactions in a safe, cost-effective, and scalable manner.

**Benefits**:

*   **Reduced Development Cost**: Test and iterate on designs without physical hardware.
*   **Improved Safety**: Simulate hazardous scenarios safely.
*   **Faster Iteration**: Quickly test new algorithms and configurations.
*   **Data Generation**: Create synthetic data for training AI models.
*   **Remote Operation & Monitoring**: Manage and control robots remotely.

## 2.2 Gazebo Fundamentals

Gazebo is a widely used 3D robotics simulator. It provides a realistic physics engine, a rich set of sensors, and a ROS 2 interface, making it an excellent tool for developing and testing robot behaviors.

### Installation:

Follow the official Gazebo installation instructions for your ROS 2 distribution. For ROS 2 Humble, you can typically install it via apt:

```bash
sudo apt update
sudo apt install ros-humble-gazebo-ros-pkgs
```

### Basic Usage:

Gazebo can be launched with a default world or a custom one. To launch with a simple world:

```bash
ros2 launch gazebo_ros empty_world_launch.py
```

This will open the Gazebo simulator with an empty environment.

## 2.3 ROS 2 Integration with Gazebo

ROS 2 packages like `gazebo_ros_pkgs` allow seamless integration between Gazebo and ROS 2. This enables you to spawn URDF/SDF models, control robots, and receive sensor data as ROS 2 topics.

### Spawning a Robot Model:

To spawn a robot model (e.g., a differential drive robot defined by a URDF file), you can use the `spawn_entity.py` ROS 2 service. A launch file is often used to automate this process.

**Example: Spawning a Differential Drive Robot**

Assume you have a URDF file named `my_robot.urdf` in your ROS 2 package's `urdf/` directory. A launch file can load this into Gazebo.

```python
# launch/spawn_robot_launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'my_robot_description' # Replace with your package name

    # Get the path to the URDF file
    robot_desc_file = os.path.join(
        get_package_share_directory(package_name),
        'urdf', 
        'my_robot.urdf'
    )

    # Launch Gazebo with an empty world
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'),
        ]),
        launch_arguments={'world': ''}.items()
    )

    # Spawn the robot model
    spawn_robot_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'my_robot', '-file', robot_desc_file],
        output='screen'
    )

    # You might also need a node to publish robot states (joint states, odometry) from Gazebo to ROS 2
    # e.g., using robot_state_publisher and joint_state_controller

    return LaunchDescription([
        gazebo_launch,
        spawn_robot_node
    ])
```

**Simulation Exercise 3.1: Differential Drive Robot in Gazebo**

1.  Create a ROS 2 package with a URDF model for a differential drive robot and a launch file similar to the example above.
2.  Ensure your URDF includes proper links, joints, and optionally, Gazebo plugins for physics and sensors (e.g., `libgazebo_ros_diff_drive.so`).
3.  Launch Gazebo with your robot:
    ```bash
source install/setup.bash
ros2 launch my_robot_description spawn_robot_launch.py
    ```
4.  You should see your robot appear in the Gazebo 3D view.

## 2.4 Unity Robotics Hub

Unity Robotics Hub is a powerful platform for creating realistic robot simulations. It leverages Unity's advanced rendering capabilities and physics engine, offering features for mixed reality, AI training, and complex environment design.

### Setup:

1.  Install Unity Hub and a compatible Unity Editor version (e.g., LTS release).
2.  Download and import the Unity Robotics package from the Unity Asset Store or GitHub.
3.  Install the ROS-TCP-Connector package to enable communication between Unity and ROS 2.

### Basic Usage:

Unity Robotics allows you to build environments, import robot models, and simulate sensors. The ROS-TCP-Connector enables ROS 2 nodes to communicate with your Unity simulation.

## 2.5 ROS-TCP-Connector

The ROS-TCP-Connector acts as a bridge, allowing ROS 2 nodes to send commands (e.g., `Twist` messages) to Unity and receive sensor data (e.g., camera images, LiDAR scans) from Unity.

**Launching the Connector**:

```bash
source install/setup.bash
ros2 run ros_tcp_endpoint roscpp_bridge --ros-args -p ROS_PORT:=10000 -p ROS_HOST:=127.0.0.1
```

## 2.6 Comparing Gazebo and Unity

| Feature           | Gazebo                                     | Unity Robotics Hub                         |
| :---------------- | :----------------------------------------- | :----------------------------------------- |
| **Physics Engine**| ODE, Bullet, DART                          | PhysX                                      |
| **Rendering**     | Basic 3D                                   | High-fidelity, realistic (URP/HDRP)        |
| **Ecosystem**     | ROS-centric, mature                        | Game dev focused, growing robotics support |
| **Ease of Use**   | Moderate (URDF/SDF, ROS integration)       | Moderate (Unity editor, C# scripting)      |
| **Use Cases**     | ROS development, basic simulation          | Photorealistic simulation, AI training, MR |
| **Hardware Req.** | Lower                                      | Higher (GPU intensive)                     |

## 2.7 Simulation Exercise 3.1 (Revisited): Differential Drive Robot in Gazebo & ROS 2 Control

Building upon the previous exercise, let's control the robot spawned in Gazebo using ROS 2.

1.  Ensure your robot URDF model includes a `diff_drive` Gazebo plugin and publishes `Odometry` messages.
2.  Run the Gazebo simulation and your robot spawn launch file.
3.  Open a new terminal, source your workspace, and run a teleoperation node:
    ```bash
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard cmd_vel:=/cmd_vel
    ```
4.  Use the keyboard (W, A, S, D) to control the robot. You should see the robot moving in Gazebo, and you can visualize its `Odometry` in RViz2.

## 2.8 Mini-Challenges

*   **Add Sensors to Gazebo Robot**: Equip your Gazebo robot with a camera and LiDAR. Visualize their output in RViz2.
*   **Import Robot to Unity**: Import your robot model into Unity and set up a basic scene. Use ROS-TCP-Connector to send commands from ROS 2 to the Unity simulation.
*   **Gazebo vs. Unity Comparison**: Discuss the pros and cons of using Gazebo vs. Unity for a specific robotics task (e.g., training a reinforcement learning agent for manipulation).

## 2.9 Conclusion

Digital twins are indispensable tools for modern robotics development. Gazebo provides a robust, ROS-centric simulation environment ideal for control systems and basic testing, while Unity offers unparalleled visual fidelity and flexibility for advanced AI training and mixed-reality applications. Mastering both will significantly enhance your ability to develop and deploy complex robotic systems.
