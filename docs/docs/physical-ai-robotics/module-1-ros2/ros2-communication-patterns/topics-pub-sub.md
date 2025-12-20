---
sidebar_position: 2
sidebar_label: Understanding Nodes, Topics, Services, and Actions
---

# Understanding Nodes, Topics, Services, and Actions in ROS 2

## Introduction to ROS 2 Architecture

The Robot Operating System 2 (ROS 2) provides a distributed computing framework that enables seamless communication between different software components of a robotic system. At the heart of this architecture are four fundamental communication patterns: Nodes, Topics, Services, and Actions. Understanding these patterns is essential for building effective robotic applications that can scale from single robots to complex multi-robot systems.

ROS 2's communication architecture is built on the Data Distribution Service (DDS) specification, which provides robust Quality of Service (QoS) policies, enabling reliable communication in mission-critical applications. Unlike ROS 1, which relied on a centralized master node, ROS 2 uses a peer-to-peer discovery mechanism that eliminates single points of failure and enables truly distributed systems.

This comprehensive guide will explore each communication pattern in depth, providing both theoretical understanding and practical implementation examples. We'll examine when to use each pattern, their performance characteristics, and best practices for implementation.

## Nodes: The Foundation of ROS 2 Systems

### What is a Node?

A Node is the fundamental execution unit in ROS 2, representing a single process that performs specific computational tasks. Nodes encapsulate functionality related to sensing, actuation, planning, control, or data processing. Each node operates as an independent process that can communicate with other nodes through the ROS 2 communication infrastructure.

The Node concept in ROS 2 is more robust than in ROS 1, with improved lifecycle management, better error handling, and enhanced resource management capabilities. Nodes can be implemented in multiple programming languages (C++, Python, Rust) and can run on different machines within the same network.

### Node Lifecycle Management

ROS 2 introduces a sophisticated lifecycle management system that allows for controlled state transitions between different operational states:

```
[Unconfigured] -> [Inactive] -> [Active] -> [Finalized]
      ^                                          |
      |------------------------------------------|
```

The lifecycle states include:
- **Unconfigured**: Initial state where components are loaded but not initialized
- **Inactive**: Components are configured but not active (for resource management)
- **Active**: Node is fully operational and processing data
- **Finalized**: Node is shutting down and releasing resources

This lifecycle management is crucial for safety-critical applications where proper initialization and shutdown procedures are essential.

### Creating Nodes in C++

Let's examine the implementation of a node in C++:

```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

class RobotController : public rclcpp::Node
{
public:
    RobotController() : Node("robot_controller")
    {
        // Initialize parameters
        this->declare_parameter("linear_speed", 0.5);
        this->declare_parameter("angular_speed", 0.5);
        
        // Create publisher
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "cmd_vel", 10);
        
        // Create subscriber
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10,
            std::bind(&RobotController::scan_callback, this, std::placeholders::_1));
        
        // Create service server
        service_ = this->create_service<std_srvs::srv::SetBool>(
            "emergency_stop",
            std::bind(&RobotController::stop_callback, this, 
                     std::placeholders::_1, std::placeholders::_2));
        
        // Create timer for periodic operations
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&RobotController::control_loop, this));
            
        RCLCPP_INFO(this->get_logger(), "Robot controller initialized");
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Process laser scan data
        double min_distance = *std::min_element(msg->ranges.begin(), msg->ranges.end());
        
        if (min_distance < 0.5) {
            RCLCPP_WARN(this->get_logger(), "Obstacle detected!");
            stop_robot();
        }
    }
    
    void stop_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                      std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        if (request->data) {
            stop_robot();
            response->success = true;
            response->message = "Robot stopped";
        } else {
            start_robot();
            response->success = true;
            response->message = "Robot started";
        }
    }
    
    void control_loop()
    {
        // Main control loop implementation
        if (!emergency_stopped_) {
            auto message = geometry_msgs::msg::Twist();
            message.linear.x = this->get_parameter("linear_speed").as_double();
            message.angular.z = 0.0; // Simplified navigation
            publisher_->publish(message);
        }
    }
    
    void stop_robot()
    {
        auto stop_msg = geometry_msgs::msg::Twist();
        publisher_->publish(stop_msg);
        emergency_stopped_ = true;
    }
    
    void start_robot()
    {
        emergency_stopped_ = false;
    }
    
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool emergency_stopped_ = false;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotController>());
    rclcpp::shutdown();
    return 0;
}
```

### Creating Nodes in Python

The Python implementation follows similar patterns but with more concise syntax:

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool
import threading
import numpy as np

class PythonRobotController(Node):
    def __init__(self):
        super().__init__('python_robot_controller')
        
        # Declare parameters with validation
        self.declare_parameter('linear_speed', 0.5, 
                               descriptor=self.create_parameter_descriptor(
                                   name='Linear velocity', 
                                   type_=rclpy.Parameter.Type.DOUBLE,
                                   range=[0.0, 2.0],
                                   step=0.1
                               ))
        self.declare_parameter('angular_speed', 0.5)
        self.declare_parameter('safety_distance', 0.5)
        
        # Get parameter values
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.safety_distance = self.get_parameter('safety_distance').value
        
        # Create QoS profiles for different communication needs
        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT  # For sensor data
        )
        cmd_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE  # For command data
        )
        
        # Create publisher for velocity commands
        self.cmd_pub = self.create_publisher(
            Twist, 
            'cmd_vel', 
            cmd_qos
        )
        
        # Create subscriber for laser scan data
        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            sensor_qos
        )
        
        # Create service for emergency stop
        self.emergency_stop_service = self.create_service(
            SetBool,
            'emergency_stop',
            self.emergency_stop_callback
        )
        
        # Create timer for periodic tasks
        self.control_timer = self.create_timer(
            0.1,  # 10 Hz control loop
            self.control_loop
        )
        
        self.emergency_stopped = False
        self.obstacle_detected = False
        self.obstacle_distance = float('inf')
        
        self.get_logger().info('Python Robot Controller initialized')
    
    def create_parameter_descriptor(self, name="", type_=None, range=None, step=None):
        """Helper to create parameter descriptors"""
        from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange
        
        descriptor = ParameterDescriptor()
        descriptor.description = name
        
        if range and step and type_ == rclpy.Parameter.Type.DOUBLE:
            float_range = FloatingPointRange()
            float_range.from_value = range[0]
            float_range.to_value = range[1]
            float_range.step = step
            descriptor.floating_point_range = [float_range]
        
        return descriptor
    
    def scan_callback(self, msg):
        """Process incoming laser scan data"""
        # Filter out invalid range readings
        valid_ranges = [r for r in msg.ranges if 0.0 < r < float('inf')]
        
        if valid_ranges:
            self.obstacle_distance = min(valid_ranges)
            self.obstacle_detected = self.obstacle_distance < self.safety_distance
        else:
            self.obstacle_distance = float('inf')
            self.obstacle_detected = False
    
    def emergency_stop_callback(self, request, response):
        """Handle emergency stop service calls"""
        self.emergency_stopped = request.data
        
        if self.emergency_stopped:
            self.get_logger().warn('Emergency stop activated!')
            self.stop_robot()
            response.success = True
            response.message = 'Robot emergency stopped'
        else:
            self.get_logger().info('Emergency stop released')
            response.success = True
            response.message = 'Robot operational'
        
        return response
    
    def control_loop(self):
        """Main control loop - runs at 10Hz"""
        if self.emergency_stopped:
            self.stop_robot()
            return
        
        if self.obstacle_detected:
            self.stop_robot()
            self.get_logger().warn(f'Obstacle at {self.obstacle_distance:.2f}m - stopping robot')
            return
        
        # Create velocity command
        cmd_msg = Twist()
        cmd_msg.linear.x = self.linear_speed
        cmd_msg.angular.z = 0.0  # No turning in this simple example
        
        self.cmd_pub.publish(cmd_msg)
    
    def stop_robot(self):
        """Stop the robot by publishing zero velocities"""
        stop_cmd = Twist()
        stop_cmd.linear.x = 0.0
        stop_cmd.angular.z = 0.0
        self.cmd_pub.publish(stop_cmd)

def main(args=None):
    rclpy.init(args=args)
    
    controller = PythonRobotController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Shutting down robot controller...')
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Topics: Asynchronous Communication Pattern

### Understanding Topic Communication

Topics enable asynchronous, decoupled communication through a publish-subscribe pattern. Publishers send messages to topics without knowledge of subscribers, while subscribers receive messages from topics without awareness of publishers. This loose coupling enables flexible system composition and resilience to component failures.

The publish-subscribe pattern is ideal for continuous data streams such as sensor readings, robot states, or control commands. Multiple publishers and subscribers can exist for the same topic, creating a distributed communication network.

### Quality of Service (QoS) Policies

ROS 2 introduces sophisticated QoS policies that control communication behavior:

```python
# Different QoS profiles for different use cases
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

# For sensor data (best effort, keep last 10 samples)
sensor_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)

# For critical commands (reliable, persistent)
command_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_ALL
)

# For low-latency applications
low_latency_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    liveliness=rclpy.qos.LivelinessPolicy.AUTOMATIC,
    deadline=Duration(seconds=0.1),  # Must be received within 100ms
    lifespan=Duration(seconds=1.0)   # Message expires after 1 second
)
```

### Advanced Topic Examples

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
import numpy as np
import time

class AdvancedTopicPublisher(Node):
    def __init__(self):
        super().__init__('advanced_topic_publisher')
        
        # Different publishers for different data types
        self.odom_publisher = self.create_publisher(
            Odometry,
            'odom',
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
                depth=1
            )
        )
        
        self.joint_publisher = self.create_publisher(
            JointState,
            'joint_states',
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_ALL,
                depth=100
            )
        )
        
        self.debug_publisher = self.create_publisher(
            Float32MultiArray,
            'debug_data',
            QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10
            )
        )
        
        # Timer for periodic publishing
        self.publish_timer = self.create_timer(0.05, self.publish_data)  # 20 Hz
        self.get_logger().info('Advanced topic publisher initialized')
    
    def publish_data(self):
        """Publish various data types with appropriate QoS"""
        # Publish odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        
        # Simulate robot movement
        current_time = self.get_clock().now().nanoseconds / 1e9
        odom_msg.pose.pose.position.x = np.sin(current_time * 0.5) * 2.0
        odom_msg.pose.pose.position.y = np.cos(current_time * 0.5) * 2.0
        odom_msg.pose.pose.position.z = 0.0
        
        # Orientation (simplified)
        odom_msg.pose.pose.orientation.z = np.sin(current_time * 0.25)
        odom_msg.pose.pose.orientation.w = np.cos(current_time * 0.25)
        
        # Twist (velocity)
        odom_msg.twist.twist.linear.x = 0.5
        odom_msg.twist.twist.angular.z = 0.25
        
        self.odom_publisher.publish(odom_msg)
        
        # Publish joint states
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = ['joint1', 'joint2', 'joint3']
        joint_msg.position = [
            np.sin(current_time), 
            np.cos(current_time * 0.5), 
            np.sin(current_time * 2.0)
        ]
        joint_msg.velocity = [
            np.cos(current_time), 
            -0.5 * np.sin(current_time * 0.5), 
            2.0 * np.cos(current_time * 2.0)
        ]
        joint_msg.effort = [0.0, 0.0, 0.0]  # Simplified
        
        self.joint_publisher.publish(joint_msg)
        
        # Publish debug data
        debug_msg = Float32MultiArray()
        debug_msg.data = [
            current_time,
            odom_msg.pose.pose.position.x,
            odom_msg.pose.pose.position.y,
            odom_msg.pose.pose.orientation.z,
            np.random.random()  # Some random metric for debugging
        ]
        self.debug_publisher.publish(debug_msg)

class AdvancedTopicSubscriber(Node):
    def __init__(self):
        super().__init__('advanced_topic_subscriber')
        
        # Subscribers with appropriate QoS matching publishers
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
                depth=1
            )
        )
        
        self.joint_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_callback,
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_ALL,
                depth=100
            )
        )
        
        self.debug_sub = self.create_subscription(
            Float32MultiArray,
            'debug_data',
            self.debug_callback,
            QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10
            )
        )
        
        # Internal state for processing
        self.robot_state = {
            'position': np.array([0.0, 0.0, 0.0]),
            'orientation': np.array([0.0, 0.0, 0.0, 1.0]),
            'joint_positions': np.array([0.0, 0.0, 0.0]),
            'last_update': 0.0
        }
        
        self.get_logger().info('Advanced topic subscriber initialized')
    
    def odom_callback(self, msg):
        """Process incoming odometry data"""
        # Extract position
        self.robot_state['position'] = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])
        
        # Extract orientation
        self.robot_state['orientation'] = np.array([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ])
        
        # Log position for debugging
        self.get_logger().debug(
            f'Position: ({self.robot_state["position"][0]:.2f}, '
            f'{self.robot_state["position"][1]:.2f}, '
            f'{self.robot_state["position"][2]:.2f})'
        )
    
    def joint_callback(self, msg):
        """Process incoming joint state data"""
        if len(msg.position) >= 3:
            self.robot_state['joint_positions'] = np.array([
                msg.position[0], msg.position[1], msg.position[2]
            ])
            
            self.get_logger().debug(
                f'Joint positions: {self.robot_state["joint_positions"]}'
            )
    
    def debug_callback(self, msg):
        """Process debug data"""
        if len(msg.data) >= 5:
            timestamp, x, y, orient_z, random_metric = msg.data[:5]
            
            # Perform some analysis on the debug data
            distance_traveled = np.sqrt(x**2 + y**2)
            
            self.get_logger().info(
                f'Time: {timestamp:.2f}, Distance: {distance_traveled:.2f}, '
                f'Random metric: {random_metric:.3f}'
            )

def run_multi_topic_example():
    """Run example with multiple topic publishers and subscribers"""
    rclpy.init()
    
    # Create publisher and subscriber nodes
    publisher = AdvancedTopicPublisher()
    subscriber = AdvancedTopicSubscriber()
    
    # Create executor to run both nodes
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(publisher)
    executor.add_node(subscriber)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        publisher.destroy_node()
        subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    run_multi_topic_example()
```

## Services: Synchronous Request-Reply Communication

### Understanding Service Communication

Services provide synchronous request-reply communication, similar to REST APIs or function calls. A service client sends a request to a service server, which processes the request and returns a response. This pattern is ideal for operations that require immediate acknowledgment and results.

Services are blocking by nature, meaning the client waits for the server's response before continuing execution. This makes services suitable for operations like triggering calibrations, querying system status, or requesting temporary behaviors.

### Service Implementation

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import Trigger, AddTwoInts, SetBool
from std_srvs.srv import Empty, SetBool as StdSetBool
import threading
import time
from typing import Tuple

class RobotServiceServer(Node):
    def __init__(self):
        super().__init__('robot_service_server')
        
        # Different service servers for various operations
        self.calibration_service = self.create_service(
            Trigger,
            'start_calibration',
            self.calibrate_callback
        )
        
        self.add_service = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_callback
        )
        
        self.emergency_service = self.create_service(
            StdSetBool,
            'emergency_stop',
            self.emergency_callback
        )
        
        self.reset_service = self.create_service(
            Empty,
            'reset_system',
            self.reset_callback
        )
        
        # Simulate robot state
        self.calibrating = False
        self.emergency_stopped = False
        self.system_reset = False
        self.system_locked = threading.Lock()
        
        self.get_logger().info('Robot service server initialized')
    
    def calibrate_callback(self, request, response):
        """Handle calibration requests"""
        self.get_logger().info('Calibration service called')
        
        # Check if already calibrating
        with self.system_locked:
            if self.calibrating:
                response.success = False
                response.message = 'Calibration already in progress'
                self.get_logger().warn('Calibration rejected - already in progress')
                return response
        
        # Perform calibration (simulated)
        self.calibrating = True
        self.get_logger().info('Starting calibration procedure...')
        
        try:
            # Simulate calibration time
            calibration_start = time.time()
            time.sleep(2.0)  # Simulate 2-second calibration process
            
            # Simulate potential calibration failure
            if time.time() % 4 < 1:  # 25% failure rate for demonstration
                response.success = False
                response.message = 'Calibration failed - sensor timeout'
                self.get_logger().error('Calibration failed')
            else:
                response.success = True
                response.message = f'Calibration completed in {time.time() - calibration_start:.2f} seconds'
                self.get_logger().info(response.message)
                
        except Exception as e:
            response.success = False
            response.message = f'Calibration failed with error: {str(e)}'
            self.get_logger().error(f'Calibration error: {str(e)}')
        finally:
            self.calibrating = False
        
        return response
    
    def add_callback(self, request, response):
        """Handle addition requests (for demonstration)"""
        self.get_logger().info(f'Adding {request.a} + {request.b}')
        
        try:
            response.sum = request.a + request.b
            response.success = True
            response.message = f'Calculation completed: {request.a} + {request.b} = {response.sum}'
        except Exception as e:
            response.success = False
            response.sum = 0
            response.message = f'Calculation failed: {str(e)}'
            self.get_logger().error(f'Addition error: {str(e)}')
        
        return response
    
    def emergency_callback(self, request, response):
        """Handle emergency stop requests"""
        self.get_logger().info(f'Emergency service called with data: {request.data}')
        
        with self.system_locked:
            self.emergency_stopped = request.data
            
            if request.data:
                response.success = True
                response.message = 'Emergency stop activated - all motions stopped'
                self.get_logger().warn('EMERGENCY STOP ACTIVATED')
            else:
                response.success = True
                response.message = 'Emergency stop released - system normal'
                self.get_logger().info('Emergency stop released')
        
        return response
    
    def reset_callback(self, request, response):
        """Handle system reset requests"""
        self.get_logger().info('System reset requested')
        
        with self.system_locked:
            # Simulate reset process
            self.get_logger().info('Resetting system components...')
            time.sleep(0.5)  # Simulate reset time
            
            # Reset states
            self.calibrating = False
            self.emergency_stopped = False
            self.system_reset = True
            
            self.get_logger().info('System reset completed')
        
        return response

class ServiceClientExample(Node):
    def __init__(self):
        super().__init__('service_client_example')
        
        # Create clients for different services
        self.calibration_client = self.create_client(Trigger, 'start_calibration')
        self.add_client = self.create_client(AddTwoInts, 'add_two_ints')
        self.emergency_client = self.create_client(StdSetBool, 'emergency_stop')
        self.reset_client = self.create_client(Empty, 'reset_system')
        
        # Wait for services to be available
        self.get_logger().info('Waiting for services...')
        while not self.calibration_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Calibration service not available, waiting...')
        
        while not self.add_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Add service not available, waiting...')
        
        while not self.emergency_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Emergency service not available, waiting...')
        
        while not self.reset_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Reset service not available, waiting...')
        
        self.get_logger().info('All services available')
        
        # Test all services
        self.test_services()
    
    def test_services(self):
        """Test all available services"""
        self.get_logger().info('Testing services...')
        
        # Test calibration service
        self.test_calibration()
        time.sleep(1)
        
        # Test addition service
        self.test_addition()
        time.sleep(1)
        
        # Test emergency stop service
        self.test_emergency_stop()
        time.sleep(1)
        
        # Test reset service
        self.test_reset()
    
    def test_calibration(self):
        """Test calibration service"""
        request = Trigger.Request()
        
        # Make service call
        future = self.calibration_client.call_async(request)
        
        # Wait for response
        rclpy.spin_until_future_complete(self, future)
        
        response = future.result()
        self.get_logger().info(f'Calibration response: {response.success}, {response.message}')
    
    def test_addition(self):
        """Test addition service"""
        request = AddTwoInts.Request()
        request.a = 10
        request.b = 25
        
        future = self.add_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        response = future.result()
        self.get_logger().info(f'Addition response: {response.success}, {response.sum}, {response.message}')
    
    def test_emergency_stop(self):
        """Test emergency stop service"""
        # Activate emergency stop
        request = StdSetBool.Request()
        request.data = True
        
        future = self.emergency_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        response = future.result()
        self.get_logger().info(f'Emergency stop response: {response.success}, {response.message}')
        
        # Release emergency stop
        request.data = False
        future = self.emergency_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        response = future.result()
        self.get_logger().info(f'Release emergency response: {response.success}, {response.message}')
    
    def test_reset(self):
        """Test system reset service"""
        request = Empty.Request()
        
        future = self.reset_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        response = future.result()
        self.get_logger().info('System reset completed')

def run_service_example():
    """Run service server and client example"""
    rclpy.init()
    
    server = RobotServiceServer()
    client = ServiceClientExample()
    
    # Run both in the same executor
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(server)
    executor.add_node(client)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        server.destroy_node()
        client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    run_service_example()
```

## Actions: Asynchronous Goal-Response Communication

### Understanding Action Communication

Actions extend the service pattern to handle long-running operations that may involve feedback and goal preemption. This pattern is essential for operations like navigation, manipulation, or complex tasks that take significant time to complete.

Actions provide several advantages over services for long-running operations:
- **Feedback**: Continuous status updates during execution
- **Goal Preemption**: Ability to cancel or replace ongoing goals
- **Result Reporting**: Structured result reporting upon completion

### Action Implementation

```python
import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Pose, Point
from nav2_msgs.action import NavigateToPose
from control_msgs.action import FollowJointTrajectory
from example_interfaces.action import Fibonacci
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
import time
import math
from typing import Optional

class NavigationActionServer(Node):
    def __init__(self):
        super().__init__('navigation_action_server')
        
        # Create action server
        self._action_server = ActionServer(
            self,
            NavigateToPose,
            'navigate_to_pose',
            execute_callback=self.execute_navigate_to_pose,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )
        
        # Robot state
        self.current_pose = Pose()
        self.current_pose.position.x = 0.0
        self.current_pose.position.y = 0.0
        self.current_pose.position.z = 0.0
        self.current_pose.orientation.x = 0.0
        self.current_pose.orientation.y = 0.0
        self.current_pose.orientation.z = 0.0
        self.current_pose.orientation.w = 1.0
        
        # Velocity publisher for movement simulation
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self.get_logger().info('Navigation action server initialized')
    
    def goal_callback(self, goal_request):
        """Accept or reject navigation goal"""
        # Check if robot is available for navigation
        # For simplicity, always accept goals
        return GoalResponse.ACCEPT
    
    def cancel_callback(self, goal_handle):
        """Accept or reject goal cancellation"""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT
    
    def execute_navigate_to_pose(self, goal_handle):
        """Execute navigation goal"""
        self.get_logger().info('Executing navigation goal')
        
        target_pose = goal_handle.request.pose
        start_pose = self.current_pose
        
        # Calculate distance to target
        distance = math.sqrt(
            (target_pose.position.x - start_pose.position.x)**2 +
            (target_pose.position.y - start_pose.position.y)**2
        )
        
        # Simulate navigation using feedback
        steps = int(distance * 10)  # 10 steps per meter
        for step in range(steps):
            # Check if goal was cancelled
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = NavigateToPose.Result()
                result.result = Pose()
                return result
            
            # Calculate current position (linear interpolation for simplicity)
            progress = (step + 1) / steps
            current_x = start_pose.position.x + (target_pose.position.x - start_pose.position.x) * progress
            current_y = start_pose.position.y + (target_pose.position.y - start_pose.position.y) * progress
            
            # Update robot pose
            self.current_pose.position.x = current_x
            self.current_pose.position.y = current_y
            
            # Calculate remaining distance
            remaining_distance = math.sqrt(
                (target_pose.position.x - current_x)**2 +
                (target_pose.position.y - current_y)**2
            )
            
            # Send feedback
            feedback_msg = NavigateToPose.Feedback()
            feedback_msg.current_pose = self.current_pose
            feedback_msg.distance_remaining = remaining_distance
            feedback_msg.progress = progress * 100.0
            
            goal_handle.publish_feedback(feedback_msg)
            
            # Simulate movement time
            time.sleep(0.1)
        
        # Navigation completed successfully
        goal_handle.succeed()
        result = NavigateToPose.Result()
        result.result = self.current_pose
        
        # Update current pose
        self.current_pose = target_pose
        
        self.get_logger().info('Navigation completed successfully')
        return result

class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')
        
        # Create action server for Fibonacci sequence generation
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            execute_callback=self.execute_fibonacci,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )
        
        self.get_logger().info('Fibonacci action server initialized')
    
    def goal_callback(self, goal_request):
        """Check if goal is valid"""
        if goal_request.order < 1:
            self.get_logger().warn('Received invalid Fibonacci order')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT
    
    def cancel_callback(self, goal_handle):
        """Accept all cancellation requests"""
        self.get_logger().info('Received Fibonacci cancellation request')
        return CancelResponse.ACCEPT
    
    def execute_fibonacci(self, goal_handle):
        """Generate Fibonacci sequence"""
        self.get_logger().info(f'Executing Fibonacci goal with order: {goal_handle.request.order}')
        
        # Generate Fibonacci sequence
        order = goal_handle.request.order
        sequence = [0, 1]
        
        for i in range(1, order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = Fibonacci.Result()
                result.sequence = sequence
                return result
            
            # Calculate next number
            next_num = sequence[-1] + sequence[-2]
            sequence.append(next_num)
            
            # Send feedback
            feedback_msg = Fibonacci.Feedback()
            feedback_msg.sequence = sequence
            goal_handle.publish_feedback(feedback_msg)
            
            # Simulate computation time
            time.sleep(0.05)
        
        # Sequence generation complete
        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = sequence[:order+1]  # Truncate to requested order
        
        self.get_logger().info(f'Fibonacci sequence generated: {result.sequence}')
        return result

class JointTrajectoryActionServer(Node):
    def __init__(self):
        super().__init__('joint_trajectory_action_server')
        
        # Create action server for joint trajectory execution
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            'follow_joint_trajectory',
            execute_callback=self.execute_follow_joint_trajectory,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )
        
        # Initialize joint positions
        self.joint_positions = {
            'joint1': 0.0,
            'joint2': 0.0,
            'joint3': 0.0
        }
        
        self.joint_pub = self.create_publisher(Float64, 'joint_position', 10)
        
        self.get_logger().info('Joint trajectory action server initialized')
    
    def goal_callback(self, goal_request):
        """Validate joint trajectory goal"""
        # Check if joint names match expected joints
        expected_joints = ['joint1', 'joint2', 'joint3']
        requested_joints = goal_request.trajectory.joint_names
        
        if set(requested_joints) != set(expected_joints):
            self.get_logger().error('Invalid joint names in trajectory')
            return GoalResponse.REJECT
        
        return GoalResponse.ACCEPT
    
    def cancel_callback(self, goal_handle):
        """Handle trajectory cancellation"""
        self.get_logger().info('Joint trajectory cancellation requested')
        return CancelResponse.ACCEPT
    
    def execute_follow_joint_trajectory(self, goal_handle):
        """Execute joint trajectory"""
        self.get_logger().info('Executing joint trajectory')
        
        trajectory = goal_handle.request.trajectory
        
        # Execute trajectory points
        for i, point in enumerate(trajectory.points):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = FollowJointTrajectory.Result()
                result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                return result
            
            # Update joint positions
            for j, joint_name in enumerate(trajectory.joint_names):
                if j < len(point.positions):
                    self.joint_positions[joint_name] = point.positions[j]
            
            # Send feedback
            feedback_msg = FollowJointTrajectory.Feedback()
            feedback_msg.joint_names = trajectory.joint_names
            feedback_msg.actual.positions = [self.joint_positions[name] for name in trajectory.joint_names]
            feedback_msg.desired.positions = point.positions
            feedback_msg.error.positions = [
                self.joint_positions[name] - point.positions[idx] 
                for idx, name in enumerate(trajectory.joint_names)
            ]
            
            goal_handle.publish_feedback(feedback_msg)
            
            # Wait for specified time
            time.sleep(point.time_from_start.sec + point.time_from_start.nanosec / 1e9)
        
        # Trajectory execution complete
        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        
        self.get_logger().info('Joint trajectory execution completed')
        return result

def run_action_servers():
    """Run all action servers"""
    rclpy.init()
    
    # Create action servers
    nav_server = NavigationActionServer()
    fib_server = FibonacciActionServer()
    traj_server = JointTrajectoryActionServer()
    
    # Combine in single executor
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(nav_server)
    executor.add_node(fib_server)
    executor.add_node(traj_server)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        nav_server.destroy_node()
        fib_server.destroy_node()
        traj_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    run_action_servers()
```

## Communication Pattern Selection Guide

### When to Use Each Pattern

Understanding when to use each communication pattern is crucial for effective system design:

1. **Topics (Publish-Subscribe)**: Use for continuous data streams like sensor readings, robot states, or broadcast messages. Best for decoupled, asynchronous communication.

2. **Services (Request-Reply)**: Use for operations requiring immediate results like triggering calibrations, querying states, or performing synchronous tasks.

3. **Actions (Goal-Oriented)**: Use for long-running operations that require feedback, preemption, or structured result reporting.

### Performance Considerations

- **Topic Latency**: Generally lowest latency for data propagation
- **Service Latency**: Higher due to request-reply overhead
- **Action Latency**: Highest due to complex state management

### Memory and CPU Usage

- **Topics**: Relatively low overhead, scales with message rate
- **Services**: Moderate overhead, scales with request rate
- **Actions**: Highest overhead, requires state management

## Advanced Patterns and Best Practices

### Hybrid Communication Patterns

Many real-world applications combine multiple communication patterns:

```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
from example_interfaces.action import NavigateToPose
from example_interfaces.srv import Trigger

class HybridRobotController(Node):
    def __init__(self):
        super().__init__('hybrid_robot_controller')
        
        # Publishers for status and commands
        self.status_pub = self.create_publisher(String, 'robot_status', 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Subscribers for sensor data
        self.sensor_sub = self.create_subscription(
            String, 'sensor_data', self.sensor_callback, 10
        )
        
        # Service for immediate commands
        self.emergency_service = self.create_service(
            Trigger, 'emergency_stop', self.emergency_stop_callback
        )
        
        # Action for long-term navigation
        self.nav_action_server = ActionServer(
            self, NavigateToPose, 'navigate_to_pose',
            execute_callback=self.navigate_execute,
            goal_callback=self.navigate_goal_callback
        )
        
        # Internal state
        self.current_state = 'IDLE'
        self.emergency_active = False
        self.navigation_active = False
        
        # Status reporting timer
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        self.get_logger().info('Hybrid robot controller initialized')
    
    def sensor_callback(self, msg):
        """Handle sensor data"""
        if self.emergency_active or self.navigation_active:
            return  # Don't process during emergency or navigation
        
        # Process sensor data and potentially change state
        if 'obstacle' in msg.data.lower():
            self.current_state = 'OBSTACLE_DETECTED'
            self.get_logger().warn('Obstacle detected from sensor')
    
    def emergency_stop_callback(self, request, response):
        """Handle emergency stop service"""
        self.emergency_active = True
        self.navigation_active = False
        self.current_state = 'EMERGENCY_STOP'
        
        # Stop robot
        stop_cmd = Twist()
        self.cmd_pub.publish(stop_cmd)
        
        response.success = True
        response.message = 'Emergency stop activated'
        
        self.get_logger().error('EMERGENCY STOP ACTIVATED')
        return response
    
    def navigate_goal_callback(self, goal_request):
        """Handle navigation goal"""
        if self.emergency_active:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT
    
    def navigate_execute(self, goal_handle):
        """Execute navigation"""
        self.navigation_active = True
        self.current_state = 'NAVIGATING'
        
        # Navigation implementation would go here
        # ...
        
        # Complete navigation
        self.navigation_active = False
        self.current_state = 'IDLE'
        
        goal_handle.succeed()
        result = NavigateToPose.Result()
        result.result = 'Navigation completed'
        return result
    
    def publish_status(self):
        """Publish robot status periodically"""
        status_msg = String()
        status_msg.data = f"{self.current_state} | Emergency: {self.emergency_active} | Nav: {self.navigation_active}"
        self.status_pub.publish(status_msg)

def main():
    rclpy.init()
    node = HybridRobotController()
    
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Integration with Real-World Applications

### Industrial Robotics

In industrial applications, the communication patterns enable:
- **Topics**: Real-time sensor fusion, machine status monitoring
- **Services**: Production line control, quality checks
- **Actions**: Complex assembly operations, inspection tasks

### Autonomous Vehicles

For autonomous driving:
- **Topics**: LiDAR and camera streams, localization updates
- **Services**: Route planning, system diagnostics
- **Actions**: Mission planning, emergency procedures

### Research Robotics

In research environments:
- **Topics**: Multi-sensor data fusion, experimental data logging
- **Services**: Calibration procedures, experimental control
- **Actions**: Complex behavioral sequences, learning episodes

## Conclusion

The communication patterns in ROS 2 - Nodes, Topics, Services, and Actions - form the fundamental building blocks for creating distributed robotic systems. Each pattern serves specific purposes and understanding their appropriate use cases is essential for effective system design.

The architecture enables flexible, scalable, and maintainable robotic applications that can evolve from simple research prototypes to complex production systems. The ability to combine these patterns creates powerful communication frameworks that can handle the diverse requirements of modern robotics applications.

In the following chapters, we'll explore advanced topics including parameter management, launch files, and complex system architectures that build upon these fundamental communication patterns.