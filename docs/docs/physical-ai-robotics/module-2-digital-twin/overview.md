---
sidebar_position: 1
sidebar_label: Overview
---

# Module 2: The Digital Twin (Gazebo & Unity) - Advanced Overview

## High-Level Overview

Digital twins represent a revolutionary approach to robotic development, providing high-fidelity virtual environments that mirror real-world conditions with exceptional accuracy. In robotics, the digital twin concept extends beyond simple visualization to encompass comprehensive modeling of physical properties, sensor characteristics, environmental interactions, and system behaviors. This enables extensive development, testing, and training activities without exposure to real-world risks, operational disruptions, or hardware damage.

Modern digital twin platforms like Gazebo and Unity offer sophisticated simulation capabilities that blend realistic physics with accurate sensor modeling, enabling roboticists to validate algorithms, train AI systems, and optimize performance before deployment. The integration of these simulation environments with ROS 2 creates a seamless development pipeline where simulation and reality share identical interfaces, message types, and communication patterns.

The critical importance of digital twins in the modern robotics workflow cannot be overstated. As robotic systems become increasingly complex, involving multiple sensors, actuators, and AI components, the ability to test and validate system behavior in a controlled, repeatable environment becomes essential. Digital twins enable:
- Rapid prototyping of algorithms and behaviors
- Comprehensive testing under diverse environmental conditions
- Safe training of machine learning models without physical risk
- Cost-effective development and debugging
- Parallel development of hardware and software components

## Deep Technical Explanation

### Gazebo Simulation System

Gazebo stands as the leading open-source robotics simulator, providing a comprehensive platform for physics-based simulation of robotic systems. Its architecture is built around multiple core components that enable realistic virtual environments.

#### Physics Engine Architecture
Gazebo supports multiple physics engines, each optimized for different simulation requirements:

- **ODE (Open Dynamics Engine):** The legacy physics engine, offering stable and deterministic simulations suitable for basic robot testing. ODE excels in scenarios with simple geometric shapes and stable contact constraints.

- **Bullet Physics:** Provides advanced collision detection and response, suitable for complex geometries and high-accuracy requirements. Bullet is particularly effective for mobile robots navigating environments with complex obstacles.

- **DART (Dynamic Animation and Robotics Toolkit):** Offers the most advanced physics simulation capabilities, supporting complex articulated bodies, soft body dynamics, and sophisticated contact mechanics. DART is ideal for humanoid robot simulation.

- **Simbody:** Stanford University's physics engine optimized for biomechanical simulation, useful for legged robot locomotion studies.

Each physics engine implements the fundamental laws of physics through numerical integration methods that calculate forces, torques, and motion over time. The choice of physics engine significantly impacts simulation fidelity, computational requirements, and stability characteristics.

#### Simulation Description Format (SDF)
SDF serves as Gazebo's primary modeling language, providing a structured XML-based approach to defining robots, environments, and simulation parameters. The SDF hierarchy includes:

**World Definition:**
```xml
<?xml version="1.0" ?>
<sdf version='1.8'>
  <world name='default'>
    <physics type='ode'>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000.0</real_time_update_rate>
    </physics>

    <light name='sun' type='directional'>
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
    </scene>
  </world>
</sdf>
```

**Robot Model Definition:**
```xml
<?xml version="1.0" ?>
<sdf version='1.8'>
  <model name='advanced_robot'>
    <link name='base_link'>
      <pose>0 0 0.5 0 0 0</pose>
      <inertial>
        <mass>10.0</mass>
        <inertia>
          <ixx>1.0</ixx>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyy>1.0</iyy>
          <iyz>0.0</iyz>
          <izz>1.0</izz>
        </inertia>
      </inertial>

      <visual name='base_visual'>
        <geometry>
          <box><size>1 1 1</size></box>
        </geometry>
        <material>
          <ambient>0.7 0.7 0.0 1</ambient>
          <diffuse>0.8 0.8 0.0 1</diffuse>
          <specular>0.1 0.1 0.1 1</specular>
        </material>
      </visual>

      <collision name='base_collision'>
        <geometry>
          <box><size>1 1 1</size></box>
        </geometry>
      </collision>

      <sensor name='camera_sensor' type='camera'>
        <pose>0.5 0 0.2 0 0 0</pose>
        <camera>
          <horizontal_fov>1.047</horizontal_fov>
          <image>
            <width>640</width>
            <height>480</height>
            <format>R8G8B8</format>
          </image>
          <clip>
            <near>0.1</near>
            <far>10.0</far>
          </clip>
        </camera>
        <always_on>1</always_on>
        <update_rate>30</update_rate>
        <visualize>true</visualize>
      </sensor>
    </link>
  </model>
</sdf>
```

#### Gazebo ROS 2 Integration
The Gazebo-ROS 2 bridge provides seamless communication between Gazebo's simulation environment and ROS 2 networks. This integration involves several key components:

**Gazebo Simulation Bridge (`ros_gz_sim`):**
This package handles the core simulation lifecycle, spawning models, controlling simulation time, and managing the simulation loop. It provides ROS 2 services for model spawning, deletion, and state control.

**Gazebo Sensors Bridge (`ros_gz_bridge`):**
This component maps Gazebo sensor types to ROS 2 message types, enabling transparent sensor data publication. For example, Gazebo's camera sensor automatically publishes to ROS 2's `sensor_msgs/Image` topic.

**Gazebo Messages (`ros_gz_interfaces`):**
Provides message definitions that bridge Gazebo-specific data types with ROS 2 standard interfaces, ensuring compatibility between simulation and real-world systems.

#### Advanced Sensor Simulation

Gazebo provides comprehensive simulation of various sensor types with realistic noise models and physical characteristics:

**Camera Sensors:**
```xml
<sensor name='stereo_camera' type='multicamera'>
  <pose>0.2 0 0.5 0 0 0</pose>
  <camera name='left_cam'>
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>1280</width>
      <height>720</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.05</near>
      <far>30.0</far>
    </clip>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.007</stddev>
    </noise>
  </camera>
  <camera name='right_cam'>
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>1280</width>
      <height>720</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.05</near>
      <far>30.0</far>
    </clip>
    <pose>0 0.12 0 0 0 0</pose> <!-- 12cm baseline -->
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.007</stddev>
    </noise>
  </camera>
  <always_on>1</always_on>
  <update_rate>30</update_rate>
  <visualize>true</visualize>
</sensor>
```

**LiDAR Sensors:**
```xml
<sensor name='velodyne_vlp16' type='ray'>
  <pose>0.3 0 0.8 0 0 0</pose>
  <ray>
    <scan>
      <horizontal>
        <samples>1800</samples>
        <resolution>1</resolution>
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>
      </horizontal>
      <vertical>
        <samples>16</samples>
        <resolution>1</resolution>
        <min_angle>-0.261799</min_angle> <!-- -15 degrees -->
        <max_angle>0.261799</max_angle>    <!--  15 degrees -->
      </vertical>
    </scan>
    <range>
      <min>0.2</min>
      <max>100.0</max>
      <resolution>0.01</resolution>
    </range>
  </ray>
  <always_on>1</always_on>
  <update_rate>10</update_rate>
  <visualize>true</visualize>
</sensor>
```

**IMU Sensors:**
```xml
<sensor name='imu_sensor' type='imu'>
  <pose>0 0 0.5 0 0 0</pose>
  <imu>
    <angular_velocity>
      <x>
        <noise type='gaussian'>
          <mean>0.0</mean>
          <stddev>0.0017</stddev>
          <bias_mean>0.0004</bias_mean>
          <bias_stddev>0.00001</bias_stddev>
        </noise>
      </x>
      <y>
        <noise type='gaussian'>
          <mean>0.0</mean>
          <stddev>0.0017</stddev>
          <bias_mean>0.0004</bias_mean>
          <bias_stddev>0.00001</bias_stddev>
        </noise>
      </y>
      <z>
        <noise type='gaussian'>
          <mean>0.0</mean>
          <stddev>0.0017</stddev>
          <bias_mean>0.0004</bias_mean>
          <bias_stddev>0.00001</bias_stddev>
        </noise>
      </z>
    </angular_velocity>
    <linear_acceleration>
      <x>
        <noise type='gaussian'>
          <mean>0.0</mean>
          <stddev>0.017</stddev>
          <bias_mean>0.01</bias_mean>
          <bias_stddev>0.001</bias_stddev>
        </noise>
      </x>
      <y>
        <noise type='gaussian'>
          <mean>0.0</mean>
          <stddev>0.017</stddev>
          <bias_mean>0.01</bias_mean>
          <bias_stddev>0.001</bias_stddev>
        </noise>
      </y>
      <z>
        <noise type='gaussian'>
          <mean>0.0</mean>
          <stddev>0.017</stddev>
          <bias_mean>0.01</bias_mean>
          <bias_stddev>0.001</bias_stddev>
        </noise>
      </z>
    </linear_acceleration>
  </imu>
  <always_on>1</always_on>
  <update_rate>100</update_rate>
  <visualize>false</visualize>
</sensor>
```

### Unity Simulation System

Unity provides a high-fidelity real-time simulation platform that excels in visual realism and interactive experiences. When combined with the Unity Robotics Hub, it becomes a powerful tool for robotic simulation with sophisticated rendering and physics capabilities.

#### Unity Robotics Hub Architecture

The Unity Robotics Hub serves as the foundational package for robotic simulation in Unity, providing:
- **ROS-TCP-Connector:** Communication bridge between Unity and ROS 2 networks
- **Robotics Library:** Pre-built components for common robotic functions
- **Simulation Tools:** Environment creation and robot simulation utilities
- **ML-Agents Integration:** Framework for training AI agents through simulation

#### Physics Engine - NVIDIA PhysX

Unity integrates NVIDIA PhysX for advanced physics simulation, offering capabilities beyond basic rigid body dynamics:
- **Soft Body Dynamics:** For simulating cloth, cables, and deformable objects
- **Vehicle Dynamics:** Specialized physics for wheeled robot simulation
- **Fluid Simulation:** For liquid handling and environmental modeling
- **Advanced Collision Detection:** Complex geometric collision scenarios

#### URDF Import and Robot Configuration

Unity provides sophisticated URDF import capabilities through the Unity Robotics package, enabling direct import of ROS-compatible robot models:

```csharp
// Example Unity C# script for importing and configuring a URDF robot
using UnityEngine;
using Unity.Robotics;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class URDFRobotController : MonoBehaviour
{
    [SerializeField] private GameObject robotRoot;
    [SerializeField] private string rosTopicName = "/joint_states";
    [SerializeField] private float publishRate = 30.0f;

    private ROSConnection ros;
    private ArticulationBody[] jointComponents;
    private float lastPublishTime;
    private int jointCount;

    void Start()
    {
        // Initialize ROS connection
        ros = ROSConnection.instance;

        // Find all articulation bodies (Unity's joint representation)
        jointComponents = robotRoot.GetComponentsInChildren<ArticulationBody>();
        jointCount = jointComponents.Length;

        lastPublishTime = Time.time;
    }

    void Update()
    {
        // Rate-limited publishing of joint states
        if (Time.time - lastPublishTime > 1.0f / publishRate)
        {
            PublishJointStates();
            lastPublishTime = Time.time;
        }
    }

    private void PublishJointStates()
    {
        // Create JointState message
        var jointState = new sensor_msgs.JointStateMsg();
        jointState.name = new string[jointCount];
        jointState.position = new double[jointCount];
        jointState.velocity = new double[jointCount];
        jointState.effort = new double[jointCount];

        // Populate joint data from Unity articulation bodies
        for (int i = 0; i < jointCount; i++)
        {
            jointState.name[i] = jointComponents[i].name;
            jointState.position[i] = jointComponents[i].jointPosition[0];
            jointState.velocity[i] = jointComponents[i].jointVelocity[0];
            jointState.effort[i] = jointComponents[i].jointForce[0];
        }

        // Set timestamp
        jointState.header = new std_msgs.HeaderMsg()
        {
            stamp = new builtin_interfaces.TimeMsg()
            {
                sec = (int)Time.time,
                nanosec = (uint)((Time.time - Mathf.Floor(Time.time)) * 1e9)
            },
            frame_id = "base_link"
        };

        // Publish to ROS
        ros.Send(rosTopicName, jointState);
    }
}
```

#### ML-Agents Integration for Reinforcement Learning

Unity ML-Agents provides a framework for training intelligent agents through reinforcement learning within simulation environments:

```csharp
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;

public class RobotAgent : Agent
{
    [SerializeField] private Transform target;
    [SerializeField] private float moveSpeed = 5.0f;
    [SerializeField] private float maxDistance = 10.0f;

    private Rigidbody rb;

    public override void Initialize()
    {
        rb = GetComponent<Rigidbody>();
    }

    public override void OnEpisodeBegin()
    {
        // Reset robot position
        rb.velocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;
        transform.position = new Vector3(
            Random.Range(-maxDistance, maxDistance),
            0.5f,
            Random.Range(-maxDistance, maxDistance)
        );

        // Reset target position
        target.position = new Vector3(
            Random.Range(-maxDistance, maxDistance),
            0.5f,
            Random.Range(-maxDistance, maxDistance)
        );
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // Agent position
        sensor.AddObservation(transform.position);

        // Target position
        sensor.AddObservation(target.position);

        // Relative position to target
        sensor.AddObservation(target.position - transform.position);

        // Current velocity
        sensor.AddObservation(rb.velocity);
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        // Get action values
        Vector3 controlSignal = Vector3.zero;
        controlSignal.x = actions.ContinuousActions[0];
        controlSignal.z = actions.ContinuousActions[1];

        // Apply movement
        rb.AddForce(controlSignal * moveSpeed);

        // Calculate distance to target
        float distanceToTarget = Vector3.Distance(transform.position, target.position);

        // Reward function
        if (distanceToTarget < 1.0f)
        {
            SetReward(1.0f);
            EndEpisode();
        }
        else
        {
            // Negative reward based on distance (closer = better reward)
            SetReward(-distanceToTarget / maxDistance);
        }

        // Punish if episode runs too long
        if (MaxStepReached())
        {
            EndEpisode();
        }
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        // Manual control for testing
        var continuousActionsOut = actionsOut.ContinuousActions;
        continuousActionsOut[0] = Input.GetAxis("Horizontal");
        continuousActionsOut[1] = Input.GetAxis("Vertical");
    }
}
```

### Advanced Simulation Techniques

#### Domain Randomization

Domain randomization is a critical technique for improving the transferability of AI models from simulation to reality by introducing controlled variations in simulation parameters:

```python
# Python script for domain randomization
import random
import numpy as np

class DomainRandomizer:
    def __init__(self):
        self.randomization_params = {
            'lighting': {
                'intensity_range': (0.5, 1.5),
                'color_temperature_range': (3000, 8000)
            },
            'material_properties': {
                'friction_range': (0.1, 1.0),
                'restitution_range': (0.0, 0.5)
            },
            'sensor_noise': {
                'camera_noise_std': (0.001, 0.01),
                'lidar_noise_std': (0.01, 0.1)
            },
            'environment_textures': {
                'textures': ['metal', 'wood', 'concrete', 'grass']
            }
        }

    def randomize_lighting(self):
        """Randomize lighting conditions"""
        intensity = random.uniform(
            self.randomization_params['lighting']['intensity_range'][0],
            self.randomization_params['lighting']['intensity_range'][1]
        )

        color_temp = random.uniform(
            self.randomization_params['lighting']['color_temperature_range'][0],
            self.randomization_params['lighting']['color_temperature_range'][1]
        )

        return {
            'intensity': intensity,
            'color_temperature': color_temp
        }

    def randomize_materials(self):
        """Randomize material properties"""
        friction = random.uniform(
            self.randomization_params['material_properties']['friction_range'][0],
            self.randomization_params['material_properties']['friction_range'][1]
        )

        restitution = random.uniform(
            self.randomization_params['material_properties']['restitution_range'][0],
            self.randomization_params['material_properties']['restitution_range'][1]
        )

        return {
            'friction': friction,
            'restitution': restitution
        }

    def randomize_sensors(self):
        """Randomize sensor parameters"""
        camera_noise = random.uniform(
            self.randomization_params['sensor_noise']['camera_noise_std'][0],
            self.randomization_params['sensor_noise']['camera_noise_std'][1]
        )

        lidar_noise = random.uniform(
            self.randomization_params['sensor_noise']['lidar_noise_std'][0],
            self.randomization_params['sensor_noise']['lidar_noise_std'][1]
        )

        return {
            'camera_noise_std': camera_noise,
            'lidar_noise_std': lidar_noise
        }

    def apply_randomization(self, simulation_env):
        """Apply randomizations to the simulation environment"""
        lighting_params = self.randomize_lighting()
        material_params = self.randomize_materials()
        sensor_params = self.randomize_sensors()

        # Apply lighting changes
        simulation_env.set_lighting_intensity(lighting_params['intensity'])
        simulation_env.set_light_color_temperature(lighting_params['color_temperature'])

        # Apply material property changes
        for material in simulation_env.materials:
            material.friction = material_params['friction']
            material.restitution = material_params['restitution']

        # Apply sensor noise
        simulation_env.set_camera_noise_std(sensor_params['camera_noise_std'])
        simulation_env.set_lidar_noise_std(sensor_params['lidar_noise_std'])

        # Apply texture randomization
        random_texture = random.choice(
            self.randomization_params['environment_textures']['textures']
        )
        simulation_env.apply_texture(random_texture)

        return {
            'lighting': lighting_params,
            'materials': material_params,
            'sensors': sensor_params,
            'texture': random_texture
        }
```

#### Multi-Sensor Data Fusion in Simulation

Simulating realistic sensor fusion requires coordination between multiple sensor types with appropriate timing, calibration, and noise characteristics:

```python
import numpy as np
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Pose, Point, Quaternion
import tf2_ros

class MultiSensorFusionSimulator:
    def __init__(self):
        # Initialize sensor simulators
        self.camera_sim = CameraSimulator()
        self.lidar_sim = LidarSimulator()
        self.imu_sim = IMUSimulator()
        self.gps_sim = GPSSimulator()

        # Initialize Kalman filter for state estimation
        self.initialize_kalman_filter()

        # TF buffer for coordinate transformation
        self.tf_buffer = tf2_ros.Buffer()

    def initialize_kalman_filter(self):
        """Initialize Kalman filter for sensor fusion"""
        # State vector: [x, y, z, vx, vy, vz, roll, pitch, yaw]
        self.state_dim = 9
        self.observation_dim = 15  # Combined sensor observations

        # Initialize state vector
        self.x = np.zeros(self.state_dim)

        # Initialize covariance matrix
        self.P = np.eye(self.state_dim) * 10.0

        # Process noise
        self.Q = np.eye(self.state_dim) * 0.1

        # Measurement noise
        self.R = np.eye(self.observation_dim) * 1.0

    def predict(self, dt):
        """Prediction step of Kalman filter"""
        # State transition matrix (simplified for example)
        F = np.eye(self.state_dim)
        F[0:3, 3:6] = np.eye(3) * dt  # Position from velocity
        # Add rotation kinematics

        # Predict state
        self.x = F @ self.x

        # Predict covariance
        self.P = F @ self.P @ F.T + self.Q

    def update(self, observation):
        """Update step of Kalman filter"""
        # Observation matrix (simplified)
        H = np.zeros((self.observation_dim, self.state_dim))
        # Fill in observation matrix based on sensor data

        # Calculate Kalman gain
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)

        # Update state
        innovation = observation - H @ self.x
        self.x = self.x + K @ innovation

        # Update covariance
        I = np.eye(self.state_dim)
        self.P = (I - K @ H) @ self.P

    def simulate_sensor_data(self, ground_truth_state):
        """Simulate realistic sensor data with noise and delays"""
        # Simulate camera data
        camera_data = self.camera_sim.get_data(ground_truth_state)
        camera_data['timestamp'] = ground_truth_state['timestamp'] + np.random.normal(0, 0.001)  # 1ms jitter
        camera_data['noise'] = np.random.normal(0, 0.01, camera_data['image'].shape)

        # Simulate LiDAR data
        lidar_data = self.lidar_sim.get_data(ground_truth_state)
        lidar_data['timestamp'] = ground_truth_state['timestamp'] + np.random.normal(0, 0.01)  # 10ms jitter
        lidar_data['noise'] = np.random.normal(0, 0.05, lidar_data['ranges'].shape)

        # Simulate IMU data
        imu_data = self.imu_sim.get_data(ground_truth_state)
        imu_data['timestamp'] = ground_truth_state['timestamp'] + np.random.normal(0, 0.0001)  # 0.1ms jitter
        imu_data['accel_noise'] = np.random.normal(0, 0.001, 3)
        imu_data['gyro_noise'] = np.random.normal(0, 0.001, 3)

        # Simulate GPS data
        gps_data = self.gps_sim.get_data(ground_truth_state)
        gps_data['timestamp'] = ground_truth_state['timestamp'] + np.random.normal(0, 0.05)  # 50ms jitter
        gps_data['horizontal_accuracy'] = np.random.exponential(2.0)  # 2m average accuracy
        gps_data['vertical_accuracy'] = np.random.exponential(5.0)   # 5m average accuracy

        return {
            'camera': camera_data,
            'lidar': lidar_data,
            'imu': imu_data,
            'gps': gps_data
        }

    def fuse_sensor_data(self, sensor_data):
        """Fuse sensor data for state estimation"""
        # Process each sensor's data based on its characteristics and timing
        fused_state = {}

        # Use IMU for high-frequency attitude and angular rate
        fused_state['attitude'] = self.process_imu_data(sensor_data['imu'])

        # Use camera and LiDAR for position and mapping
        camera_pos = self.process_camera_data(sensor_data['camera'])
        lidar_pos = self.process_lidar_data(sensor_data['lidar'])

        # Fuse position estimates using weighted average
        pos_weight_camera = 1.0 / camera_pos['uncertainty']
        pos_weight_lidar = 1.0 / lidar_pos['uncertainty']
        total_weight = pos_weight_camera + pos_weight_lidar

        fused_state['position'] = (
            camera_pos['position'] * pos_weight_camera +
            lidar_pos['position'] * pos_weight_lidar
        ) / total_weight

        # Use GPS for absolute position reference
        if sensor_data['gps']['horizontal_accuracy'] < 10.0:  # Use GPS if accurate
            fused_state['position'] = self.process_gps_data(sensor_data['gps'])

        return fused_state

class CameraSimulator:
    def __init__(self):
        # Camera intrinsic parameters
        self.fx = 640.0  # Focal length in pixels
        self.fy = 640.0
        self.cx = 320.0  # Principal point
        self.cy = 240.0
        self.width = 640
        self.height = 480
        self.k1 = -0.1  # Radial distortion
        self.k2 = 0.05
        self.p1 = 0.001  # Tangential distortion
        self.p2 = 0.002

    def get_data(self, ground_truth_state):
        """Simulate camera data"""
        # Simulate image capture based on robot's position and orientation
        position = ground_truth_state['position']
        orientation = ground_truth_state['orientation']

        # Render scene from camera viewpoint
        image = self.render_scene(position, orientation)

        # Apply noise and distortion
        image = self.apply_noise(image)
        image = self.apply_distortion(image)

        return {
            'image': image,
            'position': position,
            'orientation': orientation,
            'timestamp': ground_truth_state['timestamp']
        }

    def render_scene(self, position, orientation):
        """Render scene from camera viewpoint"""
        # In real implementation, this would use Unity or Gazebo's rendering
        # For simulation, return a placeholder with simulated features
        image = np.random.randint(0, 255, (self.height, self.width, 3), dtype=np.uint8)

        # Add simulated objects and features
        # This would be implemented based on the simulation environment
        return image

    def apply_noise(self, image):
        """Apply realistic sensor noise"""
        noise = np.random.normal(0, 10, image.shape).astype(np.int16)
        noisy_image = np.clip(image.astype(np.int16) + noise, 0, 255).astype(np.uint8)
        return noisy_image

    def apply_distortion(self, image):
        """Apply camera distortion effects"""
        # Apply radial and tangential distortion
        # This is a simplified implementation
        distorted_image = image.copy()
        # Actual distortion would involve complex coordinate transformation
        return distorted_image

class LidarSimulator:
    def __init__(self):
        self.fov_horizontal = 360  # degrees
        self.fov_vertical = 30     # degrees
        self.resolution_horizontal = 0.1  # degrees per ray
        self.resolution_vertical = 2.0    # degrees per ray
        self.max_range = 100.0     # meters
        self.min_range = 0.1       # meters

        # Calculate number of beams
        self.num_horizontal = int(self.fov_horizontal / self.resolution_horizontal)
        self.num_vertical = int(self.fov_vertical / self.resolution_vertical)
        self.ranges = np.zeros((self.num_horizontal, self.num_vertical))

    def get_data(self, ground_truth_state):
        """Simulate LiDAR data"""
        # Simulate range measurements based on environment
        ranges = self.simulate_ranges(ground_truth_state)

        # Add noise and artifacts
        ranges = self.add_noise(ranges)
        ranges = self.add_lidar_artifacts(ranges)

        return {
            'ranges': ranges,
            'intensities': np.ones_like(ranges) * 100,  # Simulated intensity
            'position': ground_truth_state['position'],
            'orientation': ground_truth_state['orientation'],
            'timestamp': ground_truth_state['timestamp']
        }

    def simulate_ranges(self, ground_truth_state):
        """Simulate LiDAR range measurements"""
        # In real implementation, this would use ray casting against the 3D environment
        # For simulation, return a placeholder with realistic range values
        ranges = np.random.uniform(
            self.min_range, self.max_range,
            (self.num_horizontal, self.num_vertical)
        )

        # Add realistic features like ground plane, obstacles
        for i in range(self.num_horizontal):
            for j in range(self.num_vertical):
                # Simulate ground plane return
                if j == self.num_vertical - 1:  # Ground plane
                    ranges[i, j] = 0.5  # Ground at 50cm below sensor

        return ranges

    def add_noise(self, ranges):
        """Add realistic LiDAR noise"""
        noise = np.random.normal(0, 0.01, ranges.shape)  # 1cm standard deviation
        noisy_ranges = ranges + noise
        return np.clip(noisy_ranges, self.min_range, self.max_range)

    def add_lidar_artifacts(self, ranges):
        """Add LiDAR-specific artifacts"""
        # Add beam divergence effects
        # Add multi-path effects
        # Add measurement dropouts
        return ranges

class IMUSimulator:
    def __init__(self):
        # IMU noise characteristics
        self.accel_noise_density = 0.017  # m/s^2/sqrt(Hz)
        self.accel_random_walk = 0.001   # m/s^3/sqrt(Hz)
        self.gyro_noise_density = 0.001  # rad/s/sqrt(Hz)
        self.gyro_random_walk = 0.0001  # rad/s^2/sqrt(Hz)

        # Bias parameters
        self.accel_bias = np.random.normal(0, 0.001, 3)
        self.gyro_bias = np.random.normal(0, 0.001, 3)

    def get_data(self, ground_truth_state):
        """Simulate IMU data"""
        # Get true acceleration and angular velocity from ground truth
        true_accel = ground_truth_state['linear_acceleration']
        true_gyro = ground_truth_state['angular_velocity']

        # Add noise and bias
        accel = true_accel + self.accel_bias + np.random.normal(0, self.accel_noise_density, 3)
        gyro = true_gyro + self.gyro_bias + np.random.normal(0, self.gyro_noise_density, 3)

        # Simulate sensor drift and temperature effects
        self.update_bias_drift()

        return {
            'linear_acceleration': accel,
            'angular_velocity': gyro,
            'timestamp': ground_truth_state['timestamp']
        }

    def update_bias_drift(self):
        """Simulate sensor bias drift over time"""
        self.accel_bias += np.random.normal(0, self.accel_random_walk, 3) * 0.01  # 10ms time step
        self.gyro_bias += np.random.normal(0, self.gyro_random_walk, 3) * 0.01

class GPSSimulator:
    def __init__(self):
        self.base_accuracy = 2.0  # Base accuracy in meters
        self.multi_path_factor = 0.5  # Multi-path effect factor
        self.sats_in_view = 12      # Number of satellites typically in view
        self.min_sats_for_fix = 4   # Minimum satellites for position fix

    def get_data(self, ground_truth_state):
        """Simulate GPS data"""
        # Get true position
        true_pos = ground_truth_state['position']

        # Calculate accuracy based on satellite geometry and environmental factors
        accuracy = self.calculate_accuracy(ground_truth_state)

        # Add position error based on accuracy
        pos_error = np.random.normal(0, accuracy/2, 2)  # 2D position error

        gps_pos = np.array([
            true_pos[0] + pos_error[0],
            true_pos[1] + pos_error[1],
            true_pos[2]  # Altitude might have different accuracy
        ])

        return {
            'position': gps_pos,
            'horizontal_accuracy': accuracy,
            'vertical_accuracy': accuracy * 1.5,  # Vertical typically worse
            'timestamp': ground_truth_state['timestamp']
        }

    def calculate_accuracy(self, ground_truth_state):
        """Calculate GPS accuracy based on satellite geometry and environment"""
        # Factors affecting GPS accuracy:
        # - Satellite geometry (DOP - Dilution of Precision)
        # - Environmental obstacles (buildings, trees)
        # - Atmospheric conditions
        # - Receiver quality

        base_accuracy = self.base_accuracy

        # Simulate environmental effects (e.g., urban canyon)
        env_multiplier = 1.0 + np.random.uniform(0, 2.0)  // 1-3x worse in poor conditions

        # Simulate satellite geometry effects
        dop_factor = np.random.uniform(1.0, 5.0)  // Good to poor geometry

        return base_accuracy * env_multiplier * dop_factor
```

## Real-World Examples

### Mars Rover Development

NASA extensively uses simulation for Mars rover development, where the cost and risk of failure are extremely high. Digital twins allow engineers to:
- Test navigation algorithms in Martian terrain simulations
- Validate autonomous decision-making capabilities
- Practice complex sample collection procedures
- Train operators without risking hardware

### Drone Delivery Systems

Companies developing drone delivery systems use sophisticated simulation environments to:
- Test flight algorithms under various weather conditions
- Validate package handling and release mechanisms
- Train AI systems for obstacle detection and avoidance
- Optimize delivery routes and scheduling

### Industrial Automation

Manufacturing companies use digital twins to:
- Design and test robot cells before physical deployment
- Optimize production line throughput and efficiency
- Train robot programs without disrupting ongoing production
- Validate safety systems and protocols

## Comprehensive Code Examples

### Advanced Gazebo Plugin for Custom Sensor

```cpp
// Advanced custom Gazebo sensor plugin
#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <sdf/sdf.hh>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace gazebo
{
  class AdvancedLidarPlugin : public SensorPlugin
  {
    public: AdvancedLidarPlugin() {}

    public: void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
    {
      // Get the parent sensor
      this->parentSensor =
        std::dynamic_pointer_cast<sensors::RaySensor>(_sensor);

      if (!this->parentSensor)
      {
        gzerr << "AdvancedLidarPlugin requires a RaySensor.\n";
        return;
      }

      // Connect to the sensor update event
      this->updateConnection = this->parentSensor->ConnectUpdated(
          std::bind(&AdvancedLidarPlugin::OnUpdate, this));

      // Initialize ROS
      if (!ros::isInitialized())
      {
        int argc = 0;
        char** argv = NULL;
        ros::init(argc, argv, "gazebo_client",
                  ros::init_options::NoSigintHandler);
      }

      this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

      // Create publisher for point cloud data
      this->pointCloudPub = this->rosNode->advertise<sensor_msgs::PointCloud2>(
          "/advanced_lidar/points", 1);

      // Parse custom parameters from SDF
      if (_sdf->HasElement("noise_std_dev"))
        this->noiseStdDev = _sdf->Get<double>("noise_std_dev");
      else
        this->noiseStdDev = 0.01;

      if (_sdf->HasElement("range_max"))
        this->rangeMax = _sdf->Get<double>("range_max");
      else
        this->rangeMax = 30.0;
    }

    private: void OnUpdate()
    {
      // Get range data from the sensor
      auto ranges = this->parentSensor->Ranges();
      auto verticalRanges = this->parentSensor->VerticalRanges();

      // Create point cloud from range data
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

      double hfov = this->parentSensor->AngleMax().Degree() -
                    this->parentSensor->AngleMin().Degree();
      double vfov = this->parentSensor->VerticalAngleMax().Degree() -
                    this->parentSensor->VerticalAngleMin().Degree();

      double angleStepH = hfov / ranges.size();
      double angleStepV = vfov / verticalRanges.size();

      for (size_t i = 0; i < ranges.size(); ++i)
      {
        for (size_t j = 0; j < verticalRanges.size(); ++j)
        {
          double range = ranges[i];
          double vRange = verticalRanges[j];

          // Add noise to range
          double noise = this->gaussianKernel(0, this->noiseStdDev);
          range += noise;

          if (range < this->rangeMax && range > 0.1)
          {
            pcl::PointXYZ point;
            double angleH = this->parentSensor->AngleMin().Radian() +
                           i * angleStepH;
            double angleV = this->parentSensor->VerticalAngleMin().Radian() +
                           j * angleStepV;

            point.x = range * cos(angleV) * cos(angleH);
            point.y = range * cos(angleV) * sin(angleH);
            point.z = range * sin(angleV);

            cloud->points.push_back(point);
          }
        }
      }

      // Convert PCL to ROS message
      sensor_msgs::PointCloud2 ros_cloud;
      pcl::toROSMsg(*cloud, ros_cloud);
      ros_cloud.header.frame_id = "advanced_lidar_frame";
      ros_cloud.header.stamp = ros::Time::now();

      // Publish point cloud
      this->pointCloudPub.publish(ros_cloud);
    }

    private: double gaussianKernel(double mu, double sigma)
    {
      // Gaussian random number generator
      static unsigned int seed = time(NULL);
      static std::default_random_engine generator(seed);
      static std::normal_distribution<double> dist(mu, sigma);

      return dist(generator);
    }

    private: std::shared_ptr<sensors::RaySensor> parentSensor;
    private: event::ConnectionPtr updateConnection;
    private: std::unique_ptr<ros::NodeHandle> rosNode;
    private: ros::Publisher pointCloudPub;
    private: double noiseStdDev;
    private: double rangeMax;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_SENSOR_PLUGIN(AdvancedLidarPlugin)
}
```

### Unity Simulation Environment with ROS Integration

```csharp
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.Geometry;

public class UnitySimulationEnvironment : MonoBehaviour
{
    [Header("ROS Connection")]
    [SerializeField] private string rosIP = "127.0.0.1";
    [SerializeField] private int rosPort = 10000;

    [Header("Environment Configuration")]
    [SerializeField] private Transform[] spawnPoints;
    [SerializeField] private GameObject[] obstaclePrefabs;
    [SerializeField] private int numberOfObstacles = 10;
    [SerializeField] private float environmentBounds = 10.0f;

    [Header("Sensor Simulation")]
    [SerializeField] private string cameraTopic = "/sim_camera/image_raw";
    [SerializeField] private string lidarTopic = "/sim_lidar/scan";
    [SerializeField] private string imuTopic = "/sim_imu/data";

    private ROSConnection ros;
    private Camera simCamera;
    private RenderTexture camRenderTexture;

    // Sensor simulation parameters
    private float cameraPublishRate = 30.0f;
    private float lidarPublishRate = 10.0f;
    private float imuPublishRate = 100.0f;

    private float lastCameraPublishTime;
    private float lastLidarPublishTime;
    private float lastImuPublishTime;

    void Start()
    {
        // Initialize ROS connection
        ros = ROSConnection.instance;
        ros.rosIPAddress = rosIP;
        ros.rosPort = rosPort;

        // Find simulation camera
        simCamera = GameObject.FindGameObjectWithTag("SimulationCamera").GetComponent<Camera>();
        if (simCamera == null)
        {
            Debug.LogError("No camera found with tag 'SimulationCamera'");
            return;
        }

        // Create render texture for camera simulation
        camRenderTexture = new RenderTexture(640, 480, 24);
        simCamera.targetTexture = camRenderTexture;

        // Initialize environment
        InitializeEnvironment();

        // Initialize timing
        lastCameraPublishTime = Time.time;
        lastLidarPublishTime = Time.time;
        lastImuPublishTime = Time.time;
    }

    void Update()
    {
        // Publish simulated sensor data
        PublishSimulatedSensors();
    }

    private void InitializeEnvironment()
    {
        // Spawn random obstacles in the environment
        for (int i = 0; i < numberOfObstacles; i++)
        {
            Vector3 spawnPosition = new Vector3(
                Random.Range(-environmentBounds, environmentBounds),
                0.5f,
                Random.Range(-environmentBounds, environmentBounds)
            );

            GameObject obstacle = Instantiate(
                obstaclePrefabs[Random.Range(0, obstaclePrefabs.Length)],
                spawnPosition,
                Quaternion.identity
            );

            // Randomize obstacle properties
            obstacle.transform.localScale = Vector3.one * Random.Range(0.5f, 2.0f);
            obstacle.GetComponent<Renderer>().material.color =
                new Color(Random.value, Random.value, Random.value);
        }
    }

    private void PublishSimulatedSensors()
    {
        // Publish camera data
        if (Time.time - lastCameraPublishTime > 1.0f / cameraPublishRate)
        {
            PublishCameraData();
            lastCameraPublishTime = Time.time;
        }

        // Publish LiDAR data
        if (Time.time - lastLidarPublishTime > 1.0f / lidarPublishRate)
        {
            PublishLidarData();
            lastLidarPublishTime = Time.time;
        }

        // Publish IMU data
        if (Time.time - lastImuPublishTime > 1.0f / imuPublishRate)
        {
            PublishImuData();
            lastImuPublishTime = Time.time;
        }
    }

    private void PublishCameraData()
    {
        // Create RenderTexture to capture camera output
        RenderTexture.active = camRenderTexture;

        // Create texture to read from RenderTexture
        Texture2D tex = new Texture2D(camRenderTexture.width, camRenderTexture.height, TextureFormat.RGB24, false);
        tex.ReadPixels(new Rect(0, 0, camRenderTexture.width, camRenderTexture.height), 0, 0);
        tex.Apply();

        // Convert texture to ROS image message
        byte[] imageData = tex.EncodeToPNG();

        // Create ROS image message
        var imageMsg = new sensor_msgs.ImageMsg();
        imageMsg.header = CreateHeader("sim_camera_optical_frame");
        imageMsg.height = (uint)camRenderTexture.height;
        imageMsg.width = (uint)camRenderTexture.width;
        imageMsg.encoding = "rgba8";  // or "rgb8" depending on format
        imageMsg.is_bigendian = 0;
        imageMsg.step = (uint)(camRenderTexture.width * 3);  // 3 bytes per pixel for RGB
        imageMsg.data = imageData;

        // Publish to ROS topic
        ros.Send(cameraTopic, imageMsg);

        // Clean up texture
        Destroy(tex);
    }

    private void PublishLidarData()
    {
        // Simulate LiDAR by casting rays in 360 degrees
        int numRays = 720; // 0.5 degree resolution
        float[] ranges = new float[numRays];
        float angleIncrement = 2 * Mathf.PI / numRays;

        Vector3 lidarPosition = transform.position; // Assume lidar is at this object's position

        for (int i = 0; i < numRays; i++)
        {
            float angle = i * angleIncrement;
            Vector3 direction = new Vector3(Mathf.Cos(angle), 0, Mathf.Sin(angle));

            // Perform raycast to find distance to nearest object
            RaycastHit hit;
            if (Physics.Raycast(lidarPosition, direction, out hit, 30.0f))
            {
                ranges[i] = hit.distance;
            }
            else
            {
                ranges[i] = 30.0f; // Max range if no hit
            }
        }

        // Create ROS LaserScan message
        var scanMsg = new sensor_msgs.LaserScanMsg();
        scanMsg.header = CreateHeader("sim_lidar_frame");
        scanMsg.angle_min = -Mathf.PI;
        scanMsg.angle_max = Mathf.PI;
        scanMsg.angle_increment = angleIncrement;
        scanMsg.time_increment = 0; // No time increment between measurements
        scanMsg.scan_time = 1.0f / lidarPublishRate;
        scanMsg.range_min = 0.1f;
        scanMsg.range_max = 30.0f;
        scanMsg.ranges = System.Array.ConvertAll(ranges, x => (float)x);

        // Add simulated noise to ranges
        for (int i = 0; i < scanMsg.ranges.Length; i++)
        {
            if (scanMsg.ranges[i] < scanMsg.range_max)
            {
                // Add Gaussian noise
                float noise = RandomGaussian(0.0f, 0.02f);
                scanMsg.ranges[i] += noise;
            }
        }

        // Publish to ROS topic
        ros.Send(lidarTopic, scanMsg);
    }

    private void PublishImuData()
    {
        // Get current acceleration and angular velocity
        // This is simulated data - in real implementation, this would come from physics
        Vector3 linearAccel = Physics.gravity + Random.insideUnitSphere * 0.1f;
        Vector3 angularVel = Random.insideUnitSphere * 0.1f;

        // Create ROS IMU message
        var imuMsg = new sensor_msgs.ImuMsg();
        imuMsg.header = CreateHeader("sim_imu_frame");

        // Set orientation (for now, identity)
        imuMsg.orientation.x = 0;
        imuMsg.orientation.y = 0;
        imuMsg.orientation.z = 0;
        imuMsg.orientation.w = 1;

        // Set angular velocity
        imuMsg.angular_velocity.x = angularVel.x;
        imuMsg.angular_velocity.y = angularVel.y;
        imuMsg.angular_velocity.z = angularVel.z;

        // Set linear acceleration
        imuMsg.linear_acceleration.x = linearAccel.x;
        imuMsg.linear_acceleration.y = linearAccel.y;
        imuMsg.linear_acceleration.z = linearAccel.z;

        // Publish to ROS topic
        ros.Send(imuTopic, imuMsg);
    }

    private std_msgs.HeaderMsg CreateHeader(string frameId)
    {
        return new std_msgs.HeaderMsg()
        {
            stamp = new builtin_interfaces.TimeMsg()
            {
                sec = (int)Time.time,
                nanosec = (uint)((Time.time - Mathf.Floor(Time.time)) * 1e9)
            },
            frame_id = frameId
        };
    }

    private float RandomGaussian(float mean, float stdDev)
    {
        // Box-Muller transform for Gaussian random number generation
        float u1 = Random.value;
        float u2 = Random.value;

        if (u1 < float.Epsilon) u1 = float.Epsilon;

        float randStdNormal = Mathf.Sqrt(-2.0f * Mathf.Log(u1)) *
                             Mathf.Sin(2.0f * Mathf.PI * u2);

        return mean + stdDev * randStdNormal;
    }
}
```

### Advanced Simulation Configuration and Management

```python
#!/usr/bin/env python3
"""
Advanced simulation configuration and management system
This module provides tools for configuring, launching, and managing
complex simulation environments for robotics research and development.
"""

import os
import yaml
import json
import subprocess
import signal
import time
import threading
from pathlib import Path
from typing import Dict, List, Optional, Any
import rospy
from std_msgs.msg import String, Float64MultiArray
from geometry_msgs.msg import Pose, Twist
from sensor_msgs.msg import LaserScan, Image, JointState
import psutil

class SimulationEnvironment:
    """Manages a complete simulation environment with multiple simulators"""

    def __init__(self, config_path: str):
        self.config = self.load_config(config_path)
        self.processes = {}
        self.is_running = False
        self.stats = {}

    def load_config(self, config_path: str) -> Dict[str, Any]:
        """Load simulation configuration from YAML file"""
        with open(config_path, 'r') as f:
            return yaml.safe_load(f)

    def start_environment(self):
        """Start all simulation components"""
        print("Starting simulation environment...")

        # Start ROS core if not running
        self._start_ros_core()

        # Start Gazebo simulation
        if self.config.get('gazebo', {}).get('enabled', True):
            self._start_gazebo()

        # Start Unity simulation if configured
        if self.config.get('unity', {}).get('enabled', False):
            self._start_unity()

        # Start custom ROS nodes
        self._start_ros_nodes()

        # Wait for systems to initialize
        time.sleep(5)

        self.is_running = True
        print("Simulation environment started successfully")

    def stop_environment(self):
        """Stop all simulation components"""
        print("Stopping simulation environment...")

        for name, proc in self.processes.items():
            if proc.poll() is None:  # Process is still running
                proc.terminate()
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    proc.kill()

        self.is_running = False
        print("Simulation environment stopped")

    def _start_ros_core(self):
        """Start ROS core if not already running"""
        try:
            # Check if roscore is already running
            subprocess.check_output(['rosnode', 'list'], timeout=1)
            print("ROS core already running")
        except (subprocess.CalledProcessError, subprocess.TimeoutExpired):
            # Start roscore in background
            roscore_cmd = ['roscore']
            roscore_proc = subprocess.Popen(roscore_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            self.processes['roscore'] = roscore_proc
            time.sleep(2)  # Allow time for ROS core to start

    def _start_gazebo(self):
        """Start Gazebo simulation"""
        gazebo_config = self.config['gazebo']
        world_file = gazebo_config.get('world_file', 'empty.world')

        gazebo_cmd = [
            'gazebo',
            f'--world={world_file}',
            f'--verbose' if gazebo_config.get('verbose', False) else ''
        ]

        # Remove empty strings from command
        gazebo_cmd = [arg for arg in gazebo_cmd if arg]

        gazebo_env = os.environ.copy()
        if gazebo_config.get('gui', True):
            gazebo_env['GAZEBO_MODEL_PATH'] = gazebo_config.get('model_path', '')
            gazebo_env['GAZEBO_RESOURCE_PATH'] = gazebo_config.get('resource_path', '')

        gazebo_proc = subprocess.Popen(gazebo_cmd, env=gazebo_env)
        self.processes['gazebo'] = gazebo_proc

    def _start_unity(self):
        """Start Unity simulation"""
        unity_config = self.config['unity']
        unity_executable = unity_config.get('executable', 'Unity')
        project_path = unity_config.get('project_path', './unity_project')

        unity_cmd = [
            unity_executable,
            '-batchmode',
            '-nographics',
            '-projectPath', project_path,
            '-executeMethod', 'SimulationStarter.StartSimulation'
        ]

        unity_proc = subprocess.Popen(unity_cmd)
        self.processes['unity'] = unity_proc

    def _start_ros_nodes(self):
        """Start custom ROS nodes"""
        if 'ros_nodes' not in self.config:
            return

        for node_config in self.config['ros_nodes']:
            node_name = node_config['name']
            node_executable = node_config['executable']
            node_args = node_config.get('args', [])

            cmd = [node_executable] + node_args
            proc = subprocess.Popen(cmd)
            self.processes[node_name] = proc

    def monitor_performance(self):
        """Monitor simulation performance metrics"""
        while self.is_running:
            # Get system resource usage
            cpu_percent = psutil.cpu_percent()
            memory_percent = psutil.virtual_memory().percent
            disk_percent = psutil.disk_usage('/').percent

            # Get process-specific metrics
            process_stats = {}
            for name, proc in self.processes.items():
                try:
                    p = psutil.Process(proc.pid)
                    process_stats[name] = {
                        'cpu_percent': p.cpu_percent(),
                        'memory_percent': p.memory_percent(),
                        'status': p.status()
                    }
                except psutil.NoSuchProcess:
                    process_stats[name] = {'status': 'exited'}

            self.stats = {
                'timestamp': time.time(),
                'system': {
                    'cpu_percent': cpu_percent,
                    'memory_percent': memory_percent,
                    'disk_percent': disk_percent
                },
                'processes': process_stats
            }

            time.sleep(1)  # Monitor every second

    def export_results(self, output_dir: str):
        """Export simulation results and statistics"""
        output_path = Path(output_dir)
        output_path.mkdir(parents=True, exist_ok=True)

        # Save configuration
        with open(output_path / 'config.yaml', 'w') as f:
            yaml.dump(self.config, f)

        # Save performance statistics
        with open(output_path / 'performance_stats.json', 'w') as f:
            json.dump(self.stats, f, indent=2)

        print(f"Results exported to {output_path}")


class SimulationScenario:
    """Represents a specific simulation scenario with test cases"""

    def __init__(self, scenario_config: Dict[str, Any]):
        self.config = scenario_config
        self.results = {}
        self.passed_tests = []
        self.failed_tests = []

    def setup_scenario(self, sim_env: SimulationEnvironment):
        """Set up the scenario in the simulation environment"""
        # Spawn robot models
        for robot in self.config.get('robots', []):
            self._spawn_robot(robot, sim_env)

        # Set up environment objects
        for obj in self.config.get('objects', []):
            self._spawn_object(obj, sim_env)

        # Configure initial conditions
        self._set_initial_conditions(sim_env)

    def run_tests(self):
        """Run all tests in the scenario"""
        for test in self.config.get('tests', []):
            test_result = self._run_single_test(test)
            if test_result['success']:
                self.passed_tests.append(test_result)
            else:
                self.failed_tests.append(test_result)

    def _spawn_robot(self, robot_config: Dict[str, Any], sim_env: SimulationEnvironment):
        """Spawn robot in simulation"""
        # Use rosrun or roslaunch to spawn robot
        spawn_cmd = [
            'rosrun', 'gazebo_ros', 'spawn_model',
            '-param', robot_config['robot_description_param'],
            '-urdf', '-model', robot_config['name']
        ]

        if 'pose' in robot_config:
            pose = robot_config['pose']
            spawn_cmd.extend(['-x', str(pose['x']), '-y', str(pose['y']), '-z', str(pose['z'])])

        subprocess.run(spawn_cmd, check=True)

    def _spawn_object(self, obj_config: Dict[str, Any], sim_env: SimulationEnvironment):
        """Spawn object in simulation"""
        # Implementation depends on object type and simulator
        pass

    def _set_initial_conditions(self, sim_env: SimulationEnvironment):
        """Set initial conditions for the scenario"""
        # Set robot positions, velocities, etc.
        pass

    def _run_single_test(self, test_config: Dict[str, Any]) -> Dict[str, Any]:
        """Run a single test case"""
        test_name = test_config['name']
        test_type = test_config['type']

        print(f"Running test: {test_name}")

        try:
            if test_type == 'navigation':
                result = self._run_navigation_test(test_config)
            elif test_type == 'manipulation':
                result = self._run_manipulation_test(test_config)
            elif test_type == 'perception':
                result = self._run_perception_test(test_config)
            else:
                raise ValueError(f"Unknown test type: {test_type}")

            return result
        except Exception as e:
            return {
                'name': test_name,
                'type': test_type,
                'success': False,
                'error': str(e),
                'timestamp': time.time()
            }

    def _run_navigation_test(self, test_config: Dict[str, Any]) -> Dict[str, Any]:
        """Run navigation test case"""
        import rospy
        from geometry_msgs.msg import PoseStamped
        from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
        import actionlib

        rospy.init_node('simulation_tester', anonymous=True)

        # Wait for move_base action server
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        client.wait_for_server()

        # Set goal
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = self._dict_to_pose(test_config['goal'])

        # Send goal
        client.send_goal(goal)

        # Wait for result
        success = client.wait_for_result(rospy.Duration(test_config.get('timeout', 60.0)))

        result = client.get_result()
        state = client.get_state()

        return {
            'name': test_config['name'],
            'type': 'navigation',
            'success': success and state == actionlib.GoalStatus.SUCCEEDED,
            'result': str(result),
            'timestamp': time.time()
        }

    def _run_manipulation_test(self, test_config: Dict[str, Any]) -> Dict[str, Any]:
        """Run manipulation test case"""
        # Implementation for manipulation tests
        # This would involve MoveIt! or custom manipulation controllers
        pass

    def _run_perception_test(self, test_config: Dict[str, Any]) -> Dict[str, Any]:
        """Run perception test case"""
        # Implementation for perception tests
        # This would involve image processing, object detection, etc.
        pass

    def _dict_to_pose(self, pose_dict: Dict[str, float]) -> Pose:
        """Convert dictionary to ROS Pose message"""
        pose = Pose()
        pose.position.x = pose_dict.get('x', 0.0)
        pose.position.y = pose_dict.get('y', 0.0)
        pose.position.z = pose_dict.get('z', 0.0)

        pose.orientation.x = pose_dict.get('qx', 0.0)
        pose.orientation.y = pose_dict.get('qy', 0.0)
        pose.orientation.z = pose_dict.get('qz', 0.0)
        pose.orientation.w = pose_dict.get('qw', 1.0)

        return pose

    def generate_report(self) -> str:
        """Generate test report"""
        report = f"""
Simulation Scenario Report
==========================
Scenario: {self.config.get('name', 'Unknown')}
Description: {self.config.get('description', 'No description')}
Total Tests: {len(self.passed_tests) + len(self.failed_tests)}
Passed: {len(self.passed_tests)}
Failed: {len(self.failed_tests)}

Passed Tests:
{chr(10).join(f"  - {test['name']}" for test in self.passed_tests)}

Failed Tests:
{chr(10).join(f"  - {test['name']}: {test.get('error', 'Unknown error')}" for test in self.failed_tests)}
        """
        return report


# Example configuration file
example_config = """
# Simulation environment configuration
name: "Advanced Robotics Simulation"
description: "Comprehensive simulation environment for robot testing"

gazebo:
  enabled: true
  world_file: "my_world.world"
  gui: true
  verbose: false
  model_path: "/path/to/models"
  resource_path: "/path/to/resources"

unity:
  enabled: false
  executable: "/path/to/unity"
  project_path: "/path/to/unity/project"

ros_nodes:
  - name: "robot_controller"
    executable: "rosrun my_package robot_controller_node"
    args: ["__name:=robot_controller"]

scenarios:
  - name: "Navigation Test"
    description: "Test robot navigation in complex environment"
    robots:
      - name: "turtlebot"
        robot_description_param: "robot_description"
        pose:
          x: 0.0
          y: 0.0
          z: 0.0
          qx: 0.0
          qy: 0.0
          qz: 0.0
          qw: 1.0

    objects:
      - name: "obstacle_1"
        type: "box"
        pose:
          x: 2.0
          y: 2.0
          size: [1.0, 1.0, 1.0]

    tests:
      - name: "Navigate to goal"
        type: "navigation"
        goal:
          x: 5.0
          y: 5.0
          z: 0.0
          qx: 0.0
          qy: 0.0
          qz: 0.0
          qw: 1.0
        timeout: 60.0
"""

if __name__ == "__main__":
    # Create example configuration file
    with open('simulation_config.yaml', 'w') as f:
        f.write(example_config)

    # Create and run simulation
    sim_env = SimulationEnvironment('simulation_config.yaml')

    # Start performance monitoring in a separate thread
    monitor_thread = threading.Thread(target=sim_env.monitor_performance, daemon=True)
    monitor_thread.start()

    try:
        # Start the simulation environment
        sim_env.start_environment()

        # Run scenarios
        config = sim_env.config
        for scenario_config in config.get('scenarios', []):
            scenario = SimulationScenario(scenario_config)
            scenario.setup_scenario(sim_env)
            scenario.run_tests()

            print(scenario.generate_report())

        # Export results
        sim_env.export_results('./results')

        # Keep running for a while to observe
        time.sleep(30)

    finally:
        sim_env.stop_environment()
```

## Simulation Exercises

### Exercise 1: Environment Creation and Custom Robot Design

**Objective:** Create a custom robot model and environment from scratch using SDF/URDF and Unity assets.

**Gazebo Component:**
1. Design a custom robot using SDF with multiple sensors (camera, LiDAR, IMU)
2. Create a complex world file with obstacles, ramps, and textured surfaces
3. Implement custom Gazebo plugins for specialized sensors or actuators
4. Integrate with ROS 2 for sensor data publication and actuator control

**Unity Component:**
1. Import the same robot model into Unity using URDF Importer
2. Create equivalent sensors using Unity's physics and rendering systems
3. Implement ROS bridge communication for sensor data and control
4. Develop a visual interface for monitoring robot status and sensor data

### Exercise 2: Multi-Sensor Data Fusion and SLAM

**Objective:** Implement sensor fusion and SLAM algorithms using simulated sensor data from both Gazebo and Unity platforms.

**Implementation Steps:**
1. Configure multiple sensors on a robot model in Gazebo (camera, LiDAR, IMU, wheel encoders)
2. Collect synchronized sensor data streams from the simulation
3. Implement Extended Kalman Filter for state estimation
4. Develop a mapping algorithm that combines different sensor modalities
5. Test robustness by introducing sensor noise and failures
6. Validate results by comparing with ground truth data from simulation

### Exercise 3: Reinforcement Learning in Simulation

**Objective:** Train a robot to perform a complex task using reinforcement learning in simulation.

**Gazebo Track:**
1. Set up a Gazebo environment with a mobile robot
2. Create obstacles and targets for navigation tasks
3. Implement reward functions for navigation and collision avoidance
4. Train policies using DQN or PPO algorithms
5. Test transfer learning from simulation to real robot (if available)

**Unity Track:**
1. Use Unity ML-Agents to create a navigation environment
2. Implement curriculum learning where task difficulty increases gradually
3. Train manipulation tasks using Unity's physics engine
4. Compare training efficiency and policy performance between Unity and Gazebo

## Hardware & Software Requirements for This Module

### Software Requirements

**Gazebo Simulation:**
- **Ubuntu 22.04 LTS** with all packages updated
- **Gazebo Garden** (latest stable version) or **Ignition Gazebo**
- **ROS 2 Humble Hawksbill** with Gazebo integration packages:
  - `ros-humble-gazebo-ros-pkgs`
  - `ros-humble-gazebo-dev`
  - `ros-humble-gazebo-plugins`
- **Development tools:** Gazebo Classic (if needed for specific packages)

**Unity Simulation:**
- **Unity Hub** with **Unity 2022.3 LTS** or later
- **Unity Robotics Hub** package (available via Package Manager)
- **ROS TCP Connector** package
- **Unity ML-Agents** package for reinforcement learning
- **Visual Studio** or **Rider** for C# development

**Additional Software:**
- **Git** for version control
- **Docker** for containerized simulation environments
- **RViz2** for visualization and debugging
- **PyCharm** or **VS Code** for Python development
- **CUDA Toolkit** (if using GPU-accelerated rendering)

### Hardware Requirements

**Minimum Configuration:**
- **CPU:** Intel i7-8700K or AMD Ryzen 7 2700X (6 cores, 12 threads)
- **RAM:** 16 GB (32 GB recommended for complex simulations)
- **GPU:** NVIDIA GTX 1060 6GB or equivalent (for Unity rendering)
- **Storage:** 256 GB SSD (for fast simulation loading)
- **Network:** Gigabit Ethernet for distributed simulation

**Recommended Configuration:**
- **CPU:** Intel i9-10900K or AMD Ryzen 9 5900X (10+ cores, 20+ threads)
- **RAM:** 32 GB (64 GB for large-scale simulations)
- **GPU:** NVIDIA RTX 3070 or higher with CUDA support
- **Storage:** 1 TB NVMe SSD for simulation assets and recordings
- **Network:** 10 Gigabit Ethernet for high-performance distributed simulation

**High-Performance Configuration:**
- **CPU:** Intel Xeon or AMD EPYC server processors
- **RAM:** 128 GB or more for large-scale multi-robot simulation
- **GPU:** NVIDIA RTX 4080/4090 or A6000 for maximum rendering performance
- **Storage:** Multiple TB of fast storage for large environments and datasets

## Mini-Tasks for Students

### Task 1: Multi-Modal Sensor Integration
Create a simulation environment with at least 4 different sensor types:
- 3D LiDAR for environment mapping
- RGB-D camera for visual perception
- IMU for orientation and acceleration
- GPS for absolute positioning

Implement sensor fusion algorithms that combine these modalities for robust state estimation.

### Task 2: Physics-Based Manipulation Simulation
Develop a simulation scenario where a robotic arm performs pick-and-place operations:
- Model realistic friction and contact physics
- Simulate object dynamics during grasping
- Implement force control for compliant grasping
- Evaluate grasping success rates under various conditions

### Task 3: Dynamic Environment Simulation
Create a simulation environment with:
- Moving obstacles that follow predefined paths
- Weather conditions (rain, fog) that affect sensor performance
- Changing lighting conditions throughout the simulation
- Destructible or deformable objects for advanced physics

### Task 4: Networked Multi-Robot Simulation
Set up a simulation with multiple robots communicating over a network:
- Implement robot-to-robot communication protocols
- Coordinate navigation and task allocation
- Handle network delays and packet loss
- Demonstrate swarm intelligence behaviors

### Task 5: Domain Randomization for Robustness
Implement domain randomization techniques to improve real-to-sim transfer:
- Randomize lighting conditions and colors
- Vary material properties and surface textures
- Add sensor noise and artifacts randomly
- Test algorithm robustness across different randomized environments

## Learning Outcomes

### Technical Skills
By completing this module, students will be able to:
1. **Design and implement complex simulation environments** for robotic systems using both Gazebo and Unity platforms.

2. **Integrate simulation environments with ROS 2** for seamless data exchange between virtual and real systems.

3. **Model realistic sensors and actuators** with appropriate noise models, latency, and failure modes.

4. **Implement advanced simulation techniques** including domain randomization, multi-sensor fusion, and physics-based interactions.

5. **Configure and manage large-scale simulation environments** with multiple robots, complex environments, and realistic scenarios.

6. **Validate simulation results** against real-world data and assess the fidelity of simulation models.

### Conceptual Understanding
Students will gain deep knowledge of:
1. **The role of simulation in the robotics development lifecycle** and its impact on development speed, safety, and cost.

2. **Physics modeling principles** and their application to robotic simulation.

3. **Sensor modeling techniques** and the trade-offs between accuracy and computational efficiency.

4. **Digital twin concepts** and their application to robotic systems development.

5. **Validation and verification methodologies** for simulation systems.

6. **Real-to-sim transfer challenges** and techniques to improve transfer success.

### Practical Competencies
Students will demonstrate:
1. **Problem-solving in simulation** by creating appropriate simulation scenarios to test robotic algorithms.

2. **Cross-platform integration** by connecting different simulation tools and real hardware.

3. **Systematic testing methodologies** using simulation to validate robotic systems.

4. **Performance optimization** for simulation systems to enable real-time operation.

5. **Scenario planning and execution** for comprehensive testing of robotic capabilities.

6. **Data analysis and interpretation** from simulation experiments to inform real-world robot design.

## Integration Points for Capstone Project

The simulation capabilities developed in this module form the backbone for the autonomous humanoid development:

### Pre-Deployment Testing
All humanoid robot behaviors will be extensively tested in simulation before physical deployment, ensuring safety and reliability through thousands of test scenarios in virtual environments.

### Training AI Components
The AI brain (Module 3) and VLA systems (Module 4) will be trained primarily in simulation environments, where they can experience a vast diversity of conditions and scenarios at minimal cost and zero risk.

### Control Algorithm Validation
Whole-body control, balance, and locomotion algorithms will be validated and refined in simulation to ensure stable and safe operation before real-world testing.

### Multi-Modal Perception
The digital twins will provide realistic sensor data streams for developing and testing perception systems that will eventually operate on the physical humanoid robot.

### Scenario-Based Development
Complex mission scenarios will be developed and refined in simulation, allowing for safe exploration of edge cases and failure conditions.

## Cross-References Between Modules

### Connection to Module 1 (ROS 2)
Module 2 simulation environments utilize the ROS 2 communication infrastructure established in Module 1, ensuring that simulated sensors publish the same message types as real sensors and simulated actuators accept the same commands as physical devices.

### Connection to Module 3 (AI Brain)
Simulation environments in Module 2 provide the training grounds for AI systems developed in Module 3, offering safe, repeatable, and controllable environments for reinforcement learning and other AI training approaches.

### Connection to Module 4 (VLA)
Vision-language-action systems benefit from the realistic visual and sensory simulation provided by Module 2, enabling training of perception-based decision-making systems before deployment to physical hardware.

## Notes for Weekly Progression (Week 1–13)

### Week 4: Gazebo Fundamentals
Introduction to basic Gazebo concepts, world creation, and simple robot models with basic sensors. Students learn to launch simulations and interact with them using ROS 2 commands.

### Week 5: Gazebo Advanced Features
Deep dive into Gazebo plugins, custom sensors, complex worlds, and physics tuning. Students implement custom sensor models and validate their behavior.

### Week 6: Unity Simulation Basics
Introduction to Unity Robotics Hub, ROS TCP Connector, and basic robot simulation in Unity. Students create simple Unity scenes and connect them to ROS 2.

### Week 7: Advanced Unity Features
Integration of ML-Agents, advanced physics, and complex sensor simulation in Unity. Students implement reinforcement learning scenarios in Unity.

### Week 8: Multi-Sensor Simulation
Focus on realistic multi-sensor simulation with proper timing, calibration, and noise modeling. Students implement sensor fusion algorithms using simulated data.

### Week 9: Domain Randomization and Robustness
Implementation of domain randomization techniques to improve sim-to-real transfer. Students develop robust algorithms that work across varied simulation conditions.

### Week 10: Performance Optimization
Optimization techniques for simulation performance, including multi-threading, rendering optimization, and physics simplification. Students profile and optimize their simulation environments.

### Week 11: Validation and Verification
Methods for validating simulation results against real-world data and ensuring simulation fidelity. Students conduct comprehensive validation of their simulation systems.

### Week 12: Integration with Other Modules
Connecting simulation environments to AI components and control algorithms from other modules. Students prepare their simulation systems for capstone project integration.

### Week 13: Capstone Preparation
Final preparation of simulation environments for the capstone project, ensuring they can support the full complexity of the autonomous humanoid system while maintaining real-time performance and realistic behavior.

This comprehensive digital twin module provides students with the essential tools and knowledge needed to develop, test, and validate robotic systems in safe, repeatable, and cost-effective virtual environments.