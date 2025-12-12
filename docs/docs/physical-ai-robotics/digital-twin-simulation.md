---
title: Module 2 - The Digital Twin (Gazebo & Unity)
sidebar_position: 3
description: Comprehensive guide to robotics simulation using Gazebo and Unity for testing and training AI systems
---

# Module 2: The Digital Twin (Gazebo & Unity)

## Overview

Robotic simulation is crucial for developing, testing, and training AI systems without the risks and costs associated with real-world deployment. A "Digital Twin" is a virtual replica of a physical system that mirrors its real-world behavior. In robotics, simulation environments allow us to:

- Test algorithms safely before deployment
- Generate synthetic training data for AI models
- Debug complex systems without hardware constraints
- Perform large-scale experiments that would be impractical in the real world

## Gazebo Simulation Framework

Gazebo is a physics-based simulation environment that provides realistic interaction between robots and their environment. It uses a physics engine to simulate gravity, friction, collision, and other physical phenomena.

### Core Components

1. **SDF (Simulation Description Format)**: XML-based format for defining robots and environments
2. **Physics Engines**: ODE, Bullet, DART, and Simbody for realistic physics simulation
3. **Sensors**: Cameras, LIDAR, IMU, contact sensors, and more
4. **ROS 2 Integration**: Gazebo ROS 2 packages for seamless communication

### Example SDF Robot Model

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="simple_robot">
    <!-- Base link -->
    <link name="base_link">
      <inertial>
        <mass>1.0</mass>
        <inertia>
          <ixx>0.01</ixx> <ixy>0.0</ixy> <ixz>0.0</ixz>
          <iyy>0.01</iyy> <iyz>0.0</iyz>
          <izz>0.01</izz>
        </inertia>
      </inertial>
      
      <visual name="visual">
        <geometry>
          <box><size>0.2 0.2 0.2</size></box>
        </geometry>
        <material>
          <script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <name>Gazebo/Blue</name>
          </script>
        </material>
      </visual>
      
      <collision name="collision">
        <geometry>
          <box><size>0.2 0.2 0.2</size></box>
        </geometry>
      </collision>
    </link>
    
    <!-- Arm link -->
    <link name="arm_link">
      <inertial>
        <mass>0.5</mass>
        <inertia>
          <ixx>0.001</ixx> <ixy>0.0</ixy> <ixz>0.0</ixz>
          <iyy>0.001</iyy> <iyz>0.0</iyz>
          <izz>0.001</izz>
        </inertia>
      </inertial>
      
      <visual name="arm_visual">
        <geometry>
          <cylinder><radius>0.02</radius><length>0.3</length></cylinder>
        </geometry>
      </visual>
      
      <collision name="arm_collision">
        <geometry>
          <cylinder><radius>0.02</radius><length>0.3</length></cylinder>
        </geometry>
      </collision>
    </link>
    
    <!-- Joint connecting base and arm -->
    <joint name="arm_joint" type="revolute">
      <parent>base_link</parent>
      <child>arm_link</child>
      <axis>
        <xyz>0 0 1</xyz>
        <limit><lower>-1.57</lower><upper>1.57</upper></limit>
      </axis>
      <pose>0.2 0 0 0 0 0</pose>
    </joint>
  </model>
</sdf>
```

### Gazebo ROS 2 Integration

```python
# gazebo_robot_controller.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Image
from cv_bridge import CvBridge
import cv2

class GazeboRobotController(Node):
    def __init__(self):
        super().__init__('gazebo_robot_controller')
        
        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscriber for laser scan data
        self.laser_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10
        )
        
        # Subscriber for camera data
        self.camera_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.camera_callback,
            10
        )
        
        self.bridge = CvBridge()
        self.timer = self.create_timer(0.1, self.control_loop)
        
    def laser_callback(self, msg):
        # Process laser scan data for obstacle detection
        min_range = min(msg.ranges)
        if min_range < 0.5:  # Obstacle within 0.5m
            self.avoid_obstacle()
            
    def camera_callback(self, msg):
        # Process camera image
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # Example: detect red objects
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lower_red = (0, 50, 50)
        upper_red = (10, 255, 255)
        mask = cv2.inRange(hsv, lower_red, upper_red)
        
        # Process the mask to find red objects
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 100:  # Ignore small contours
                # Object detected, log or take action
                self.get_logger().info(f'Red object detected with area: {area}')
    
    def control_loop(self):
        # Simple control logic
        msg = Twist()
        msg.linear.x = 0.2  # Move forward at 0.2 m/s
        msg.angular.z = 0.0  # No rotation
        self.cmd_vel_pub.publish(msg)
        
    def avoid_obstacle(self):
        # Simple obstacle avoidance
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.5  # Turn right
        self.cmd_vel_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    controller = GazeboRobotController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Unity Simulation Framework

Unity provides a high-fidelity simulation environment with advanced rendering capabilities and the Unity Robotics Hub for ROS communication. It's particularly useful for:

- High-quality visual perception training
- Complex environment modeling
- Virtual reality integration
- Advanced rendering for synthetic data generation

### Unity ROS TCP Connector

Unity can communicate with ROS 2 systems using the ROS TCP Connector:

```csharp
// RobotController.cs for Unity
using UnityEngine;
using RosSharp.RosBridgeClient;
using RosSharp.Messages.Geometry;

public class RobotController : MonoBehaviour
{
    private RosSocket rosSocket;
    private string robotName = "UnityRobot";
    
    void Start()
    {
        // Connect to ROS bridge
        rosSocket = new RosSocket(new RosBridgeClient.Protocols.WebSocketNetProtocol("ws://localhost:9090"));
        
        // Subscribe to velocity commands
        rosSocket.Subscribe<Twist>($"/{robotName}/cmd_vel", ReceiveVelocityCommand);
    }
    
    void ReceiveVelocityCommand(Twist twist)
    {
        // Apply linear and angular velocities to robot
        Vector3 linearVelocity = new Vector3((float)twist.linear.x, (float)twist.linear.y, (float)twist.linear.z);
        Vector3 angularVelocity = new Vector3((float)twist.angular.x, (float)twist.angular.y, (float)twist.angular.z);
        
        // Update robot position based on received velocities
        transform.Translate(linearVelocity * Time.deltaTime);
        transform.Rotate(angularVelocity * Mathf.Rad2Deg * Time.deltaTime);
    }
    
    void OnDestroy()
    {
        rosSocket.Close();
    }
}
```

## Simulation Architecture

```mermaid
graph TD
    A[Physical Robot] -->|Sensors & Actuators| B(Real World)
    C[Digital Twin (Gazebo/Unity)] -->|Simulated Sensors & Actuators| D(Simulated World)
    B --- F[ROS 2 Control]
    D --- F
    F --> E{AI Brain}
    
    subgraph Simulation Environment
        C
        D
    end
    
    subgraph ROS 2 System
        F
        E
    end
```

## Practical Simulation Steps

### Setting up Gazebo Environment

1. **Install Gazebo Garden (or Fortress)**
```bash
sudo apt update
sudo apt install gazebo
```

2. **Launch Gazebo with a basic world**
```bash
gazebo --verbose worlds/empty.sdf
```

3. **Spawn a custom robot into the world**
```bash
# Save the SDF above as simple_robot.sdf
gz sim -r -v 4 simple_robot.sdf
```

4. **Connect to ROS 2**
```bash
# For Gazebo Garden
sudo apt install ros-humble-ros-gz
```

5. **Bridge sensor data to ROS 2 topics**
```bash
# Bridge camera data
ros2 run ros_gz_image image_bridge /camera/image_raw

# Bridge laser scan data
ros2 run ros_gz_bridge parameter_bridge /scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan
```

### Unity Simulation Setup

1. **Install Unity Hub and Unity 2022.3 LTS**
2. **Install Unity Robotics Hub** from the Unity Asset Store
3. **Import ROS TCP Connector** package
4. **Create a simple scene with a robot model**
5. **Configure communication with ROS 2**

## Context7 Integration in Simulation

Simulation environments can benefit from Context7 documentation access:

```python
# simulation_context_helper.py
import rclpy
from rclpy.node import Node
import subprocess
import json

class SimulationContextHelper(Node):
    def __init__(self):
        super().__init__('simulation_context_helper')
        
    async def get_gazebo_model_docs(self, model_name):
        """
        Retrieve documentation for Gazebo models using MCP
        """
        try:
            # Use MCP to retrieve model documentation
            # This would connect to a Context7 server via MCP
            mcp_result = await self.query_context7_mcp({
                'library': 'gazebo',
                'topic': f'model_{model_name}',
                'action': 'get_documentation'
            })
            return mcp_result
        except Exception as e:
            self.get_logger().error(f'Error retrieving Gazebo model docs: {e}')
            return None
    
    async def get_unity_component_docs(self, component_type):
        """
        Retrieve documentation for Unity components
        """
        try:
            # Use MCP to retrieve Unity component documentation
            mcp_result = await self.query_context7_mcp({
                'library': 'unity',
                'topic': f'component_{component_type}',
                'action': 'get_documentation'
            })
            return mcp_result
        except Exception as e:
            self.get_logger().error(f'Error retrieving Unity component docs: {e}')
            return None
    
    async def query_context7_mcp(self, query_params):
        """
        Query Context7 documentation server via MCP protocol
        """
        # This is a conceptual implementation
        # Actual implementation would use MCP client
        pass

def main(args=None):
    rclpy.init(args=args)
    node = SimulationContextHelper()
    # Additional implementation would go here
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Simulation Best Practices

### Physics Accuracy
- Use appropriate friction and collision models
- Set realistic mass and inertia properties
- Validate simulation behavior against real robots when possible

### Sensor Simulation
- Model sensor noise and limitations
- Include latency and bandwidth constraints
- Validate sensor models with real-world data

### Performance Optimization
- Simplify complex geometries for collision detection
- Use appropriate update rates for different components
- Consider Level of Detail (LOD) for complex environments

## Mini-Tasks for Students

1. Create a custom Gazebo world with obstacles and a simple robot model
2. Implement a ROS 2 node that receives camera data from Gazebo and performs basic image processing
3. Develop a Unity scene with a robot that can receive movement commands via ROS TCP connector
4. Create a simulation scenario where a robot must navigate to a goal while avoiding obstacles
5. Implement a perception pipeline that processes simulated sensor data to detect and classify objects

## Integration Points for Capstone Project

Simulation environments will be critical for the capstone project:
- Extensive development and testing of the autonomous humanoid
- Training of control policies for locomotion and manipulation
- Testing in diverse and complex scenarios
- Validation of perception systems before real-world deployment
- Generation of synthetic training data for vision and VLA systems

## Learning Outcomes

After completing this module, students should be able to:
1. Understand the principles and benefits of robotic simulation
2. Develop and deploy robot models and environments in Gazebo using SDF
3. Integrate Gazebo with ROS 2 for sensor data and motor control
4. Set up Unity projects for robotic simulation and integrate with ROS 2
5. Design and implement perception pipelines in simulated environments
6. Evaluate simulation fidelity by comparing with real-world behavior
7. Use Context7 documentation to enhance simulation development

## Weekly Progression Notes

**Weeks 4-5**: Focus on Gazebo basics, ROS 2 integration, and custom world creation. Students should be able to spawn their own robot models and implement basic sensor processing.

**Week 6**: Introduction to Unity simulation, ROS TCP connector setup, and basic Unity-ROS communication. Students should create a simple Unity environment with ROS integration.

## Hardware & Software Requirements

### Software Stack
- Ubuntu 22.04 LTS
- ROS 2 Humble
- Gazebo Garden/Fortress
- Unity Hub with Unity 2022.3 LTS
- Unity Robotics Hub package
- Python 3.10+ with rclpy

### Hardware Requirements
- High-performance CPU (Intel i7/AMD Ryzen 7 or better)
- NVIDIA GPU (RTX 3060 or equivalent recommended)
- 16GB+ RAM (32GB+ recommended for complex scenes)
- High-speed SSD for simulation data