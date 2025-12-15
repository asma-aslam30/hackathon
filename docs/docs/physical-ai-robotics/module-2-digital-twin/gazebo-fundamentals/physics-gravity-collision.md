---
sidebar_position: 1
sidebar_label: Gazebo Simulation - Physics, Gravity, and Collision Modeling
---

# Gazebo Simulation - Physics, Gravity, and Collision Modeling with Context7 Integration

## Table of Contents
1. [Introduction](#introduction)
2. [Deep Technical Analysis](#deep-technical-analysis)
3. [Gazebo Architecture and Physics Engine](#gazebo-architecture-and-physics-engine)
4. [Gravity and Environmental Modeling](#gravity-and-environmental-modeling)
5. [Collision Detection and Response](#collision-detection-and-response)
6. [Advanced Simulation Techniques](#advanced-simulation-techniques)
7. [Context7 Integration for Documentation](#context7-integration-for-documentation)
8. [Simulation Optimization](#simulation-optimization)
9. [Real-World Applications](#real-world-applications)
10. [Future Developments](#future-developments)
11. [Summary](#summary)

## Introduction

Gazebo stands as one of the most prominent simulation environments in robotics, providing a physically accurate platform for testing, developing, and validating robotic systems before deployment in the real world. For humanoid robots and complex robotic applications, Gazebo offers sophisticated physics simulation capabilities that include realistic modeling of gravitational forces, collision detection, and multi-body dynamics. These capabilities are essential for developing robots that must interact with the physical world in meaningful ways.

The simulation of physics, gravity, and collisions in Gazebo is achieved through highly optimized physics engines, including ODE (Open Dynamics Engine), Bullet, and DART (Dynamic Animation and Robotics Toolkit). These engines provide the computational foundation for modeling complex interactions that are fundamental to robotic operations, from simple object manipulation to complex locomotion patterns in humanoid robots.

The integration of Context7 documentation systems enhances the Gazebo development process by providing developers with immediate access to up-to-date physics parameters, simulation best practices, and configuration recommendations. This integration streamlines the development workflow and ensures that simulation parameters align with current best practices.

This comprehensive guide explores the latest developments in Gazebo simulation as of 2025, focusing on physics simulation, gravitational modeling, and collision detection. We examine how Context7 integration can enhance the simulation development process, providing developers with immediate access to relevant documentation and configuration guidance.

## Deep Technical Analysis

### Physics Engine Fundamentals

Gazebo's core functionality relies on sophisticated physics engines that model the interaction of physical bodies through mathematical representations of real-world physics. The physics simulation encompasses several fundamental domains:

1. **Rigid Body Dynamics**: Modeling the motion and interaction of inflexible bodies
2. **Collision Detection**: Identifying when and where physical bodies intersect
3. **Contact Dynamics**: Calculating the forces resulting from collisions
4. **Constraint Solving**: Managing mathematical constraints like joints
5. **Integration Methods**: Numerical methods for solving equations of motion

The physics simulation in Gazebo is typically performed at high frequencies (1000+ Hz) to ensure numerical stability and accurate representation of physical phenomena. This high-frequency simulation is decoupled from the visualization and control loops, allowing for detailed physics calculations without impacting real-time performance requirements.

### Gravitational Field Modeling

Gravitational simulation in Gazebo is implemented through a constant downward acceleration vector, typically set to 9.81 m/s² to match Earth's gravitational acceleration. However, the system can be configured for different gravitational environments, making it suitable for simulating robots operating in various conditions, from Earth to the Moon to microgravity environments.

The gravitational field affects:
- All rigid bodies with mass
- Pendulum-like motions and balance
- Friction and contact forces
- Fluid-like interactions (in advanced configurations)

### Collision Detection Systems

Gazebo implements hierarchical collision detection using:
1. **Broad Phase**: Fast approximation using bounding volume hierarchies (BVH)
2. **Narrow Phase**: Precise collision detection between potentially intersecting bodies
3. **Contact Resolution**: Calculation of collision forces and response

The system supports various geometric representations for collision detection:
- Primitive shapes (spheres, boxes, cylinders)
- Triangle meshes for complex geometries
- Heightmaps for terrain simulation

### Mathematical Foundation

The physics simulation in Gazebo is based on classical mechanics and numerical integration methods:

- **Newton-Euler Equations**: For rigid body motion
- **Lagrange Equations**: For constrained systems
- **Runge-Kutta Methods**: For numerical integration
- **Gauss-Seidel Methods**: For constraint solving
- **Linear Complementarity Problem (LCP)**: For contact resolution

These mathematical foundations enable the accurate simulation of complex multi-body systems typical in humanoid robotics.

## Gazebo Architecture and Physics Engine

### Core Architecture Components

Gazebo's architecture is built around several key components that work together to provide a comprehensive simulation environment:

```xml
<?xml version="1.0"?>
<sdf version="1.9">
  <world name="default">
    <!-- Physics engine configuration -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <gravity>0 0 -9.8</gravity>
    </physics>
    
    <!-- Lighting and environmental settings -->
    <light name="sun" type="directional">
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
      <direction>-0.1 0.2 -1.0</direction>
    </light>
    
    <!-- Ground plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>1.0</mu>
                <mu2>1.0</mu2>
                <fdir1>0 0 0</fdir1>
                <slip1>0</slip1>
                <slip2>0</slip2>
              </ode>
              <torsional>
                <coefficient>1.0</coefficient>
                <patch_radius>0</patch_radius>
                <surface_radius>0</surface_radius>
                <use_patch_radius>false</use_patch_radius>
                <ode>
                  <slip>0</slip>
                </ode>
              </torsional>
            </friction>
            <contact>
              <ode>
                <soft_cfm>0</soft_cfm>
                <soft_erp>0.2</soft_erp>
                <kp>1e+13</kp>
                <kd>1.0</kd>
                <max_vel>0.01</max_vel>
                <min_depth>0</min_depth>
              </ode>
            </contact>
            <bounce>
              <restitution_coefficient>0</restitution_coefficient>
              <threshold>100000</threshold>
            </bounce>
          </surface>
        </collision>
        <visual name="visual">
          <cast_shadows>false</cast_shadows>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.3 0.3 0.3 1</ambient>
            <diffuse>0.7 0.7 0.7 1</diffuse>
            <specular>0.3 0.3 0.3 1</specular>
          </material>
        </visual>
      </link>
    </model>
    
    <!-- Example robot model -->
    <model name="simple_robot">
      <pose>0 0 0.5 0 0 0</pose>
      <link name="chassis">
        <inertial>
          <mass>10.0</mass>
          <inertia>
            <ixx>0.4</ixx>
            <ixy>0.0</ixy>
            <ixz>0.0</ixz>
            <iyy>0.4</iyy>
            <iyz>0.0</iyz>
            <izz>0.2</izz>
          </inertia>
        </inertial>
        <collision name="collision">
          <geometry>
            <box>
              <size>1.0 0.5 0.3</size>
            </box>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>0.5</mu>
                <mu2>0.5</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1.0 0.5 0.3</size>
            </box>
          </geometry>
          <material>
            <ambient>0.0 0.8 0.0 1</ambient>
            <diffuse>0.0 0.8 0.0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

### Physics Engine Configuration

Gazebo supports multiple physics engines, each with its own strengths and capabilities. The choice of physics engine can significantly impact simulation accuracy, performance, and stability:

#### ODE (Open Dynamics Engine)
```xml
<physics type="ode">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1</real_time_factor>
  <real_time_update_rate>1000</real_time_update_rate>
  <gravity>0 0 -9.8</gravity>
  <ode>
    <solver>
      <type>quick</type>
      <iters>10</iters>
      <sor>1.3</sor>
    </solver>
    <constraints>
      <cfm>0.0</cfm>
      <erp>0.2</erp>
      <contact_max_correcting_vel>100</contact_max_correcting_vel>
      <contact_surface_layer>0.001</contact_surface_layer>
    </constraints>
  </ode>
</physics>
```

#### Bullet Physics Engine
```xml
<physics type="bullet">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1</real_time_factor>
  <real_time_update_rate>1000</real_time_update_rate>
  <gravity>0 0 -9.8</gravity>
  <bullet>
    <solver>
      <type>sequential_impulse</type>
      <iterations>10</iterations>
      <sor>1.3</sor>
    </solver>
    <constraints>
      <cfm>0.0</cfm>
      <erp>0.2</erp>
    </constraints>
  </bullet>
</physics>
```

### Model Definition and Physics Properties

The SDF (Simulation Description Format) model definition includes detailed physics properties for each component:

```xml
<model name="advanced_robot">
  <link name="base_link">
    <inertial>
      <!-- Mass properties -->
      <mass>50.0</mass>
      <pose>0 0 0.5 0 0 0</pose>
      <inertia>
        <ixx>3.0</ixx>
        <ixy>0.0</ixy>
        <ixz>0.0</ixz>
        <iyy>3.0</iyy>
        <iyz>0.0</iyz>
        <izz>5.0</izz>
      </inertia>
    </inertial>
    
    <!-- Collision properties -->
    <collision name="base_collision">
      <pose>0 0 0.5 0 0 0</pose>
      <geometry>
        <mesh>
          <uri>model://advanced_robot/meshes/base_link_collision.dae</uri>
        </mesh>
      </geometry>
      <surface>
        <!-- Friction properties -->
        <friction>
          <ode>
            <mu>0.8</mu>
            <mu2>0.8</mu2>
            <fdir1>0 0 0</fdir1>
            <slip1>0.0</slip1>
            <slip2>0.0</slip2>
          </ode>
          <torsional>
            <coefficient>1.0</coefficient>
            <patch_radius>0.01</patch_radius>
            <surface_radius>0.02</surface_radius>
            <use_patch_radius>false</use_patch_radius>
            <ode>
              <slip>0.5</slip>
            </ode>
          </torsional>
        </friction>
        
        <!-- Contact properties -->
        <contact>
          <ode>
            <soft_cfm>0.0</soft_cfm>
            <soft_erp>0.2</soft_erp>
            <kp>1000000000000.0</kp>
            <kd>1.0</kd>
            <max_vel>100.0</max_vel>
            <min_depth>0.001</min_depth>
          </ode>
        </contact>
        
        <!-- Bounce properties -->
        <bounce>
          <restitution_coefficient>0.1</restitution_coefficient>
          <threshold>100000</threshold>
        </bounce>
      </surface>
    </collision>
    
    <!-- Visual properties -->
    <visual name="base_visual">
      <pose>0 0 0.5 0 0 0</pose>
      <geometry>
        <mesh>
          <uri>model://advanced_robot/meshes/base_link_visual.dae</uri>
        </mesh>
      </geometry>
      <material>
        <script>
          <uri>file://media/materials/scripts/gazebo.material</uri>
          <name>Gazebo/Orange</name>
        </script>
      </material>
    </visual>
  </link>
</model>
```

### Joint Definition with Physics Properties

Joints in Gazebo define the kinematic relationships between links and include detailed actuator and constraint properties:

```xml
<joint name="hip_joint" type="revolute">
  <parent>base_link</parent>
  <child>left_leg</child>
  <pose>0.0 -0.2 0.0 0 0 0</pose>
  <axis>
    <xyz>1 0 0</xyz>
    <limit>
      <lower>-1.57</lower>
      <upper>1.57</upper>
      <effort>1000.0</effort>
      <velocity>2.0</velocity>
    </limit>
    <dynamics>
      <damping>0.1</damping>
      <friction>0.05</friction>
    </dynamics>
  </axis>
  
  <!-- Joint transmission for actuator control -->
  <transmission name="hip_transmission">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="hip_joint">
      <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
    </joint>
    <actuator name="hip_motor">
      <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
      <mechanicalReduction>1</mechanicalReduction>
    </actuator>
  </transmission>
</joint>
```

## Gravity and Environmental Modeling

### Gravitational Field Configuration

Gazebo allows for detailed configuration of gravitational fields, enabling simulation in different environments and conditions. The gravitational field is defined at the world level and affects all dynamic objects in the simulation:

```xml
<world name="earth_environment">
  <physics type="ode">
    <!-- Standard Earth gravity -->
    <gravity>0 0 -9.8</gravity>
    <!-- Physics engine properties -->
    <max_step_size>0.001</max_step_size>
    <real_time_factor>1</real_time_factor>
    <real_time_update_rate>1000</real_time_update_rate>
  </physics>
</world>

<world name="moon_environment">
  <physics type="ode">
    <!-- Moon gravity (1/6 of Earth) -->
    <gravity>0 0 -1.625</gravity>
    <!-- Physics engine properties -->
    <max_step_size>0.001</max_step_size>
    <real_time_factor>1</real_time_factor>
    <real_time_update_rate>1000</real_time_update_rate>
  </physics>
</world>

<world name="custom_gravity_field">
  <physics type="ode">
    <!-- Custom gravity vector (e.g., for tilted planet or accelerating vehicle) -->
    <gravity>-0.5 0 -9.3</gravity>
    <!-- This represents a 0.5 m/s² acceleration in the -X direction and 9.3 m/s² in the -Z direction -->
    <max_step_size>0.001</max_step_size>
    <real_time_factor>1</real_time_factor>
    <real_time_update_rate>1000</real_time_update_rate>
  </physics>
</world>
```

### Environmental Forces and Effects

Beyond basic gravitational forces, Gazebo can simulate additional environmental forces that affect robotic systems:

```xml
<model name="environmental_effects_model">
  <!-- Include plugins for environmental effects -->
  <plugin filename="libgazebo_ros_force.so" name="gazebo_ros_force">
    <!-- Plugin for applying forces to models -->
  </plugin>
  
  <!-- Wind simulation -->
  <plugin filename="libgazebo_ros_wind.so" name="gazebo_ros_wind">
    <always_on>true</always_on>
    <pub_topic>/gazebo/wind</pub_topic>
    <wind_direction>1 0 0</wind_direction>
    <wind_force>0.5 0 0</wind_force>
    <wind_gust_enabled>true</wind_gust_enabled>
    <wind_gust_start>10 0 0</wind_gust_start>
    <wind_gust_duration>5</wind_gust_duration>
    <wind_gust_force>2 0 0</wind_gust_force>
  </plugin>
</model>
```

### Terrain and Surface Modeling

Complex terrain and surface interactions are crucial for humanoid robot simulation, particularly for bipedal locomotion and navigation:

```xml
<model name="complex_terrain">
  <static>true</static>
  <link name="terrain_link">
    <collision name="terrain_collision">
      <geometry>
        <!-- Heightmap for terrain -->
        <heightmap>
          <uri>file://media/materials/textures/terrain_heightmap.png</uri>
          <size>100 100 20</size>
          <pos>0 0 0</pos>
        </heightmap>
      </geometry>
      <surface>
        <friction>
          <ode>
            <mu>0.8</mu>
            <mu2>0.8</mu2>
          </ode>
        </friction>
        <bounce>
          <restitution_coefficient>0.1</restitution_coefficient>
        </bounce>
      </surface>
    </collision>
    <visual name="terrain_visual">
      <geometry>
        <heightmap>
          <uri>file://media/materials/textures/terrain_texture.png</uri>
          <size>100 100 20</size>
          <pos>0 0 0</pos>
        </heightmap>
      </geometry>
    </visual>
  </link>
</model>
```

### Fluid Simulation Integration

For robots that operate in or around fluids, Gazebo provides limited fluid simulation capabilities:

```xml
<!-- Example water simulation -->
<model name="water_plane">
  <static>true</static>
  <link name="water_surface">
    <collision name="water_collision">
      <geometry>
        <plane>
          <normal>0 0 1</normal>
          <size>100 100</size>
        </plane>
      </geometry>
      <surface>
        <friction>
          <ode>
            <mu>0.1</mu>
            <mu2>0.1</mu2>
          </ode>
        </friction>
        <contact>
          <ode>
            <kd>100</kd>  <!-- High damping for fluid-like behavior -->
            <kp>1000000</kp>
          </ode>
        </contact>
      </surface>
    </collision>
    <visual name="water_visual">
      <geometry>
        <plane>
          <normal>0 0 1</normal>
          <size>100 100</size>
        </plane>
      </geometry>
      <material>
        <script>
          <uri>file://media/materials/scripts/gazebo.material</uri>
          <name>Gazebo/BlueLaser</name>
        </script>
      </material>
    </visual>
  </link>
</model>
```

## Collision Detection and Response

### Collision Geometry Types

Gazebo supports various geometric representations for collision detection, each with different performance and accuracy characteristics:

#### Primitive Shapes
```xml
<!-- Box collision -->
<collision name="box_collision">
  <geometry>
    <box>
      <size>0.5 0.3 0.2</size>
    </box>
  </geometry>
</collision>

<!-- Sphere collision -->
<collision name="sphere_collision">
  <geometry>
    <sphere>
      <radius>0.1</radius>
    </sphere>
  </geometry>
</collision>

<!-- Cylinder collision -->
<collision name="cylinder_collision">
  <geometry>
    <cylinder>
      <radius>0.05</radius>
      <length>0.2</length>
    </cylinder>
  </geometry>
</collision>

<!-- Capsule collision (not directly supported, using two spheres and a cylinder) -->
<collision name="capsule_collision_1">
  <geometry>
    <sphere>
      <radius>0.05</radius>
    </sphere>
  </geometry>
  <pose>0 0 0.1 0 0 0</pose>
</collision>
<collision name="capsule_collision_2">
  <geometry>
    <cylinder>
      <radius>0.05</radius>
      <length>0.1</length>
    </cylinder>
    <pose>0 0 0.05 0 0 0</pose>
</collision>
```

#### Mesh-Based Collisions
```xml
<collision name="complex_collision">
  <geometry>
    <mesh>
      <uri>model://robot/meshes/complex_shape.stl</uri>
      <scale>1.0 1.0 1.0</scale>
    </mesh>
  </geometry>
  <max_contacts>10</max_contacts>
  <surface>
    <contact>
      <ode>
        <max_vel>100</max_vel>
        <min_depth>0.001</min_depth>
      </ode>
    </contact>
  </surface>
</collision>
```

### Collision Detection Configuration

Collision detection performance can be optimized through various configuration parameters:

```xml
<collision name="optimized_collision">
  <geometry>
    <mesh>
      <uri>model://robot/meshes/complex_shape.stl</uri>
    </mesh>
  </geometry>
  
  <!-- Limit contact points for performance -->
  <max_contacts>10</max_contacts>
  
  <surface>
    <!-- Contact dynamics -->
    <contact>
      <ode>
        <!-- Contact correction velocity (higher = more forceful correction) -->
        <max_vel>100.0</max_vel>
        <!-- Minimum depth before contact is detected (lower = more accurate but slower) -->
        <min_depth>0.001</min_depth>
        
        <!-- Soft CFM and ERP for stability -->
        <soft_cfm>0.0</soft_cfm>
        <soft_erp>0.2</soft_erp>
        
        <!-- Spring and damper coefficients -->
        <kp>1000000000000.0</kp>
        <kd>1.0</kd>
      </ode>
    </contact>
    
    <!-- Friction properties -->
    <friction>
      <ode>
        <!-- Primary friction coefficient -->
        <mu>0.8</mu>
        <!-- Secondary friction coefficient -->
        <mu2>0.8</mu2>
        <!-- Friction direction -->
        <fdir1>0 0 0</fdir1>
        <!-- Slip parameters -->
        <slip1>0.0</slip1>
        <slip2>0.0</slip2>
      </ode>
      
      <!-- Torsional friction -->
      <torsional>
        <coefficient>1.0</coefficient>
        <patch_radius>0.01</patch_radius>
        <surface_radius>0.02</surface_radius>
        <use_patch_radius>false</use_patch_radius>
        <ode>
          <slip>0.5</slip>
        </ode>
      </torsional>
    </friction>
    
    <!-- Bounce properties -->
    <bounce>
      <restitution_coefficient>0.1</restitution_coefficient>
      <threshold>100000</threshold>
    </bounce>
  </surface>
</collision>
```

### Multi-Body Dynamics and Constraints

Complex humanoid robots require sophisticated constraint handling for proper joint behavior:

```xml
<!-- Ball and socket joint simulation using multiple constraints -->
<model name="humanoid_leg">
  <!-- Thigh link -->
  <link name="thigh">
    <inertial>
      <mass>3.0</mass>
      <inertia>
        <ixx>0.05</ixx>
        <ixy>0.0</ixy>
        <ixz>0.0</ixz>
        <iyy>0.05</iyy>
        <iyz>0.0</iyz>
        <izz>0.01</izz>
      </inertia>
    </inertial>
  </link>
  
  <!-- Calf link -->
  <link name="calf">
    <inertial>
      <mass>2.0</mass>
      <inertia>
        <ixx>0.03</ixx>
        <ixy>0.0</ixy>
        <ixz>0.0</ixz>
        <iyy>0.03</iyy>
        <iyz>0.0</iyz>
        <izz>0.01</izz>
      </inertia>
    </inertial>
  </link>
  
  <!-- Knee joint constraint -->
  <joint name="knee_joint" type="revolute">
    <parent>thigh</parent>
    <child>calf</child>
    <axis>
      <xyz>0 1 0</xyz>
      <limit>
        <lower>0</lower>
        <upper>2.5</upper>
        <effort>200</effort>
        <velocity>5</velocity>
      </limit>
      <dynamics>
        <damping>1.0</damping>
        <friction>0.1</friction>
      </dynamics>
    </axis>
    <pose>0 0 -0.4 0 0 0</pose>
  </joint>
</model>
```

### Collision Filtering and Groups

For complex simulations, collision filtering allows specific objects to ignore collisions with others:

```xml
<!-- Define collision groups -->
<state world_name="default">
  <sim_time>0 0</sim_time>
  <real_time>0 0</real_time>
  <wall_time>0 0</wall_time>
  <model name="robot">
    <link name="base_link">
      <!-- Collision bitmask to enable/disable collisions -->
      <collision name="base_collision">
        <geometry>
          <box>
            <size>0.5 0.5 0.5</size>
          </box>
        </geometry>
        <!-- Bitmask for collision filtering -->
        <surface>
          <contact>
            <collide_without_contact>false</collide_without_contact>
          </contact>
        </surface>
      </collision>
    </link>
  </model>
</state>
```

## Advanced Simulation Techniques

### High-Fidelity Physics Tuning

Achieving high-fidelity physics simulation requires careful tuning of multiple parameters:

```xml
<!-- World with high-fidelity physics settings -->
<world name="high_fidelity_world">
  <physics type="ode">
    <!-- Small time steps for accuracy -->
    <max_step_size>0.0005</max_step_size>
    <!-- High update rate -->
    <real_time_update_rate>2000</real_time_update_rate>
    <!-- Real-time factor of 1.0 for real-time simulation -->
    <real_time_factor>1.0</real_time_factor>
    <!-- Gravity -->
    <gravity>0 0 -9.8</gravity>
    
    <ode>
      <!-- Solver configuration for accuracy -->
      <solver>
        <type>quick</type>
        <!-- Higher number of iterations for better constraint satisfaction -->
        <iters>100</iters>
        <!-- SOR over-relaxation parameter -->
        <sor>1.3</sor>
      </solver>
      
      <!-- Constraint configuration -->
      <constraints>
        <!-- Constraint Force Mixing - lower values for stiffer constraints -->
        <cfm>1e-10</cfm>
        <!-- Error Reduction Parameter - higher values for faster error correction -->
        <erp>0.99</erp>
        <!-- Maximum correcting velocity -->
        <contact_max_correcting_vel>100</contact_max_correcting_vel>
        <!-- Contact surface layer (penetration tolerance) -->
        <contact_surface_layer>0.0001</contact_surface_layer>
      </constraints>
    </ode>
  </physics>
  
  <!-- Include plugins for high-fidelity sensor simulation -->
  <include>
    <uri>model://sun</uri>
  </include>
</world>
```

### Multi-Physics Simulation

For complex humanoid robots, combining multiple physics domains can provide more realistic simulations:

```xml
<!-- Example of combining different physics effects -->
<model name="multi_physics_robot">
  <!-- Rigid body dynamics for structure -->
  <link name="torso">
    <inertial>
      <mass>15.0</mass>
      <inertia>
        <ixx>0.8</ixx>
        <ixy>0.01</ixy>
        <ixz>0.02</ixz>
        <iyy>0.7</iyy>
        <iyz>0.01</iyz>
        <izz>0.5</izz>
      </inertia>
    </inertial>
    <collision>
      <geometry>
        <box>
          <size>0.4 0.3 0.6</size>
        </box>
      </geometry>
    </collision>
  </link>
  
  <!-- Flexible elements modeled as rigid bodies with springs -->
  <link name="flexible_component">
    <inertial>
      <mass>0.5</mass>
      <inertia>
        <ixx>0.001</ixx>
        <ixy>0</ixy>
        <ixz>0</ixz>
        <iyy>0.001</iyy>
        <iyz>0</iyz>
        <izz>0.001</izz>
      </inertial>
    </inertial>
  </link>
  
  <!-- Joint with spring-damper characteristics -->
  <joint name="flexible_joint" type="revolute">
    <parent>torso</parent>
    <child>flexible_component</child>
    <axis>
      <xyz>0 0 1</xyz>
      <limit>
        <lower>-0.1</lower>
        <upper>0.1</upper>
        <effort>10</effort>
        <velocity>1</velocity>
      </limit>
      <!-- Spring-damper dynamics -->
      <dynamics>
        <spring_reference>0</spring_reference>
        <spring_stiffness>1000</spring_stiffness>
        <damping>50</damping>
        <friction>0.1</friction>
      </dynamics>
    </axis>
  </joint>
  
  <!-- Plugin for custom physics effects -->
  <plugin name="custom_physics_plugin" filename="libcustom_physics.so">
    <robot_name>multi_physics_robot</robot_name>
    <physics_parameters>
      