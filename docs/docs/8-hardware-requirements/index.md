# Module 8: Hardware Requirements

This module details the essential hardware components required for undertaking the Physical AI and Humanoid Robotics textbook. It covers compute platforms, sensors, actuators, and robot platforms, providing guidance on selection and basic setup.

**Learning Outcomes**:

*   Identify key compute platforms suitable for robotics development (e.g., NVIDIA Jetson, powerful desktops).
*   Understand the role and types of common robotic sensors (e.g., cameras, IMUs, LiDAR).
*   Learn about actuators and motor control for robot movement.
*   Familiarize with popular robot platforms and chassis.
*   Consider power requirements and management for robotic systems.

## 8.1 Compute Platforms

Choosing the right compute platform is crucial for handling the computational demands of AI and robotics tasks. 

### 8.1.1 NVIDIA Jetson Series

*   **Overview**: Embedded computing platforms designed by NVIDIA for AI and robotics. They offer a powerful GPU for accelerated AI processing, making them ideal for edge computing scenarios.
*   **Popular Models**: Jetson Nano, Jetson Xavier NX, Jetson Orin Nano/NX.
*   **Use Cases**: Onboard robot processing, real-time perception, AI inference.
*   **Considerations**: Power consumption, available I/O ports, software compatibility (JetPack).

### 8.1.2 Desktop Workstations

*   **Overview**: High-performance PCs with powerful CPUs and GPUs. Suitable for development, simulation, and tasks requiring maximum computational power.
*   **Use Cases**: Training large AI models, running complex simulations, heavy data processing.
*   **Considerations**: Cost, power requirements, portability (lack thereof).

## 8.2 Robotic Sensors

Sensors provide robots with information about their environment and internal state. 

### 8.2.1 Cameras

*   **Types**: RGB cameras, Depth cameras (Stereo, Time-of-Flight), Event cameras.
*   **Functionality**: Visual perception, object recognition, SLAM, depth sensing.
*   **Examples**: Intel RealSense (D400 series), ZED stereo cameras, standard USB webcams.
*   **Key Specs**: Resolution, frame rate, depth accuracy, field of view.

### 8.2.2 Inertial Measurement Units (IMUs)

*   **Types**: Accelerometers, Gyroscopes, Magnetometers.
*   **Functionality**: Measure orientation, angular velocity, and linear acceleration. Crucial for state estimation and IMU fusion.
*   **Examples**: BNO055, MPU6050.
*   **Key Specs**: Sensitivity, bias, noise levels, integration capabilities.

### 8.2.3 LiDAR (Light Detection and Ranging)

*   **Types**: 2D LiDAR, 3D LiDAR.
*   **Functionality**: Measure distance by emitting laser pulses, used for mapping, localization, and obstacle avoidance.
*   **Examples**: RPLIDAR, Velodyne, Ouster.
*   **Key Specs**: Range, angular resolution, scan rate, accuracy.

## 8.3 Actuators and Motor Control

Actuators are the components that enable a robot to move and interact with its environment. 

### 8.3.1 Motors

*   **Types**: DC Motors (brushed, brushless), Servo Motors, Stepper Motors.
*   **Functionality**: Provide rotational or linear motion.
*   **Considerations**: Torque, speed, precision, power requirements, feedback mechanisms (encoders).

### 8.3.2 Motor Drivers

*   **Functionality**: Interface between the robot's controller (e.g., Jetson GPIO) and the motors. They regulate motor speed and direction.
*   **Examples**: L298N (simple DC motor driver), DRV8825 (stepper motor driver), dedicated ESCs (Electronic Speed Controllers) for brushless motors.

## 8.4 Robot Platforms and Chassis

Choosing a robot chassis provides the mechanical structure for mounting components.

### 8.4.1 DIY Mobile Robot Chassis

*   **Components**: Base frame, wheels, motors, motor mounts.
*   **Advantages**: Cost-effective, customizable, good for learning fundamentals.
*   **Considerations**: Material (plastic, metal, 3D printed), wheel type, size, weight capacity.

### 8.4.2 Commercial Robot Platforms

*   **Examples**: Unitree (Robotics), Clearpath Robotics, TurtleBot.
*   **Advantages**: Integrated systems, robust design, often come with ROS support.
*   **Considerations**: Cost, specific capabilities, compatibility.

## 8.5 Power Systems

Providing adequate and stable power is critical for robot operation.

### 8.5.1 Batteries

*   **Types**: LiPo (Lithium Polymer), Li-ion, NiMH, Lead-Acid.
*   **Key Metrics**: Voltage (V), Capacity (mAh or Ah), Discharge Rate (C-rating for LiPo).
*   **Safety**: Proper handling, charging, and storage are essential, especially for LiPo batteries.

### 8.5.2 Power Distribution

*   **Functionality**: Regulators (e.g., Buck converters) to step down voltage for different components (sensors, Jetson module).
*   **Considerations**: Current handling capacity, efficiency, heat dissipation.

## 8.6 Mini-Challenges

*   **Component Research**: Research three different types of depth cameras and compare their specifications for a given robotics task (e.g., indoor navigation).
*   **Power Budget Calculation**: Estimate the power requirements for a simple robot (e.g., Jetson, two DC motors, a camera) and select an appropriate battery and voltage regulators.
*   **Sensor Interfacing Discussion**: Discuss the trade-offs between using an IMU directly connected to a microcontroller vs. an IMU with a ROS 2 interface board.

## 8.7 Conclusion

Understanding the hardware ecosystem is fundamental to building functional robots. This module has provided an overview of the key components, from compute and sensors to actuators and power. The next modules will build upon this knowledge by integrating these components with software frameworks like ROS 2, digital twins, and advanced AI techniques.
