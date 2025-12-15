---
sidebar_position: 4
sidebar_label: ROS 2 Topics and Services - Deep Dive
---

# ROS 2 Topics and Services - A Comprehensive Guide with Context7 Integration

## Table of Contents
1. [Introduction](#introduction)
2. [Deep Technical Analysis](#deep-technical-analysis)
3. [ROS 2 Topics](#ros-2-topics)
4. [ROS 2 Services](#ros-2-services)
5. [Advanced Communication Patterns](#advanced-communication-patterns)
6. [Quality of Service (QoS) in Depth](#quality-of-service-qos-in-depth)
7. [ROS 2 Actions](#ros-2-actions)
8. [Context7 Integration for Documentation](#context7-integration-for-documentation)
9. [Practical Examples](#practical-examples)
10. [Best Practices and Troubleshooting](#best-practices-and-troubleshooting)
11. [Real-World Applications](#real-world-applications)
12. [Future Developments](#future-developments)
13. [Summary](#summary)

## Introduction

The Robot Operating System 2 (ROS 2) represents a fundamental shift in how robotic systems communicate and coordinate. At the heart of ROS 2's communication architecture are three primary patterns: Topics for asynchronous, one-way communication; Services for synchronous, request-response communication; and Actions for complex, long-running tasks with feedback. These communication patterns enable distributed robotic systems to work together seamlessly, forming what is often termed the "robotic nervous system."

ROS 2 Topics and Services have evolved significantly since the original ROS 1, addressing critical issues of scalability, real-time performance, and security that are essential for modern robotic applications. These communication patterns are built on the Data Distribution Service (DDS) middleware, which provides robust, high-performance communication capabilities suitable for both simulation and real-world robotic systems.

This comprehensive guide explores the latest developments in ROS 2 communication patterns as of 2025, including recent improvements in Quality of Service (QoS) policies, security features, and integration with modern development practices. We'll also examine how Context7 integration can enhance the developer experience by providing on-demand access to up-to-date documentation and best practices.

## Deep Technical Analysis

### Evolution from ROS 1 to ROS 2 Communication

The communication architecture in ROS 2 represents a complete redesign from ROS 1. While ROS 1 relied on a centralized master architecture that could create bottlenecks and single points of failure, ROS 2 employs a distributed architecture built on DDS (Data Distribution Service) middleware. This change enables:

1. **Decentralized Architecture**: No central master node that can become a single point of failure
2. **Native Security**: Built-in security mechanisms at the middleware level
3. **Quality of Service (QoS) Policies**: Configurable behavior for different types of data
4. **Multi-platform Support**: Native support for various operating systems and architectures
5. **Real-time Performance**: Deterministic behavior suitable for safety-critical applications

### Core Communication Concepts

In ROS 2, communication is built around three main patterns:

**Topics** implement a publish-subscribe model where publishers send messages to topics and subscribers receive messages from topics. This pattern is asynchronous and allows for multiple publishers and subscribers to the same topic. Topics are ideal for streaming data like sensor readings, robot states, or control commands that don't require confirmation of receipt.

**Services** implement a request-response model where a client sends a request to a service and waits for a response. This pattern is synchronous and typically involves a single service server handling requests from multiple clients. Services are well-suited for operations that require immediate results, such as triggering an action or querying system status.

**Actions** extend the service pattern for long-running operations that require feedback during execution and the ability to cancel operations. Actions are perfect for complex tasks like robot navigation or manipulation where you need to monitor progress and potentially interrupt the process.

### Middleware Abstraction Layer

ROS 2 introduces the concept of a middleware abstraction layer that allows the core ROS 2 API to work with different middleware implementations. While DDS is the primary middleware, this architecture enables:

- **Pluggable Middleware**: Different DDS vendors can be used (Fast DDS, Cyclone DDS, RTI Connext DDS)
- **Custom Middleware**: Organizations can implement custom middleware for specialized needs
- **Middleware Performance Tuning**: Different middleware implementations can be optimized for specific use cases

## ROS 2 Topics

### Understanding Topics in Depth

Topics form the backbone of ROS 2's data distribution system. They enable asynchronous, one-way communication between nodes and are essential for streaming data throughout a robotic system. A topic is essentially a named channel through which data is transmitted from publishers to subscribers.

The publish-subscribe pattern provides several key advantages:

1. **Decoupling**: Publishers and subscribers don't need to know about each other's existence
2. **Scalability**: Multiple publishers and subscribers can operate on the same topic
3. **Asynchronous Communication**: Publishers can send messages without waiting for processing
4. **Broadcast Nature**: Messages sent to a topic are automatically received by all subscribers

### Topic Implementation and Architecture

The implementation of topics in ROS 2 involves several layers:

1. **Application Layer**: The user's ROS 2 code using publishers and subscribers
2. **ROS 2 Client Library (rclpy/rclcpp)**: Provides the Python/C++ APIs for topic operations
3. **ROS 2 Client Library (rcl)**: Lower-level C library that abstracts middleware operations
4. **Middleware Interface (rmw)**: Interface that allows switching between different middleware implementations
5. **DDS Implementation**: The actual middleware that handles message distribution

This layered architecture allows for flexibility while maintaining performance and reliability.

### Creating Publishers and Subscribers

Let's explore the implementation patterns for creating robust publishers and subscribers:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Float32
from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import Twist, Pose
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

class RobustTopicNode(Node):
    """
    Example node demonstrating various publisher and subscriber patterns
    with appropriate QoS configurations for different use cases
    """
    
    def __init__(self):
        super().__init__('robust_topic_node')
        
        # High-frequency sensor data (e.g., laser scanner)
        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST
        )
        self.laser_pub = self.create_publisher(LaserScan, 'laser_scan', sensor_qos)
        self.laser_sub = self.create_subscription(
            LaserScan, 'laser_scan', self.laser_callback, sensor_qos
        )
        
        # Control commands (frequent, need to be reliable)
        cmd_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST
        )
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', cmd_qos)
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, cmd_qos
        )
        
        # Robot state (frequently updated, may be missed occasionally)
        state_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST
        )
        self.robot_state_pub = self.create_publisher(Pose, 'robot_pose', state_qos)
        
        # Status messages (less critical, can tolerate some loss)
        status_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST
        )
        self.status_pub = self.create_publisher(String, 'robot_status', status_qos)
        
        # Create a timer for periodic state updates
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz
        self.i = 0
        
        self.get_logger().info('Robust Topic Node initialized')
    
    def laser_callback(self, msg):
        """Handle incoming laser scan data"""
        self.get_logger().debug(f'Received laser scan with {len(msg.ranges)} ranges')
        
        # Process the laser data (simplified example)
        if msg.ranges:
            min_range = min(msg.ranges)
            if min_range < 0.5:  # Obstacle within 0.5m
                self.get_logger().warn(f'Obstacle detected at {min_range:.2f}m')
    
    def cmd_vel_callback(self, msg):
        """Handle incoming velocity commands"""
        self.get_logger().info(f'Received velocity command: {msg.linear.x}, {msg.angular.z}')
        
        # In a real robot, this would send commands to the motor controller
        # For now, just log the command
        self.get_logger().info(f'Velocity command processed')
    
    def timer_callback(self):
        """Publish periodic messages"""
        # Publish robot state
        msg = Pose()
        msg.position.x = float(self.i)
        msg.position.y = 0.0
        msg.position.z = 0.0
        msg.orientation.w = 1.0
        self.robot_state_pub.publish(msg)
        
        # Publish status message
        status_msg = String()
        status_msg.data = f'Robot operational at step {self.i}'
        self.status_pub.publish(status_msg)
        
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    node = RobustTopicNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Advanced Topic Patterns

#### Topic Bridges
Topic bridges enable communication between different ROS networks or between ROS and non-ROS systems:

```python
import rclpy
from rclpy.node import Node
import socket
import json
from std_msgs.msg import String

class TopicBridgeNode(Node):
    """
    Example of a topic bridge that connects ROS 2 to external systems
    """
    
    def __init__(self):
        super().__init__('topic_bridge_node')
        
        # ROS 2 publisher and subscriber
        self.internal_pub = self.create_publisher(String, 'internal_topic', 10)
        self.internal_sub = self.create_subscription(
            String, 'internal_topic', self.internal_callback, 10
        )
        
        # Socket for external communication
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(('localhost', 12345))
        self.socket.settimeout(0.1)  # Non-blocking with timeout
        
        # Timer to check for external messages
        self.timer = self.create_timer(0.01, self.check_external_messages)
        
        self.get_logger().info('Topic Bridge Node initialized')
    
    def internal_callback(self, msg):
        """Forward internal ROS 2 message to external system"""
        try:
            # Send to external system (simplified example)
            external_msg = {
                'source': 'ros2',
                'topic': 'internal_topic',
                'data': msg.data,
                'timestamp': self.get_clock().now().nanoseconds
            }
            json_msg = json.dumps(external_msg)
            self.socket.sendto(json_msg.encode(), ('localhost', 12346))
        except Exception as e:
            self.get_logger().error(f'Error sending to external system: {e}')
    
    def check_external_messages(self):
        """Check for messages from external system"""
        try:
            data, addr = self.socket.recvfrom(1024)
            external_msg = json.loads(data.decode())
            
            # Publish to internal ROS 2 topic
            ros_msg = String()
            ros_msg.data = f"External: {external_msg['data']}"
            self.internal_pub.publish(ros_msg)
            
        except socket.timeout:
            # No message received, continue
            pass
        except json.JSONDecodeError:
            self.get_logger().warn('Received invalid JSON from external system')
        except Exception as e:
            self.get_logger().error(f'Error processing external message: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = TopicBridgeNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Bridge node interrupted')
    finally:
        node.socket.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Topic Monitoring and Diagnostics
Monitoring topics for performance and health is crucial for robotic systems:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from rclpy.time import Time
from collections import deque
import statistics

class TopicMonitorNode(Node):
    """
    Node that monitors topic performance and provides diagnostics
    """
    
    def __init__(self):
        super().__init__('topic_monitor_node')
        
        # Topic monitoring data
        self.message_times = deque(maxlen=100)  # Keep last 100 message times
        self.message_counts = {}  # Count messages per topic
        
        # Subscribers for topics to monitor
        self.monitored_sub = self.create_subscription(
            String, 'monitored_topic', self.monitored_callback, 10
        )
        
        # Diagnostic publisher
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 1)
        
        # Timer to publish diagnostics
        self.diag_timer = self.create_timer(1.0, self.publish_diagnostics)
        
        self.get_logger().info('Topic Monitor Node initialized')
    
    def monitored_callback(self, msg):
        """Monitor callback performance"""
        current_time = self.get_clock().now()
        self.message_times.append(current_time.nanoseconds)
        
        # Update message count for this topic
        topic_name = 'monitored_topic'
        if topic_name not in self.message_counts:
            self.message_counts[topic_name] = 0
        self.message_counts[topic_name] += 1
    
    def publish_diagnostics(self):
        """Publish diagnostic information"""
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()
        
        # Calculate diagnostic information
        if len(self.message_times) > 1:
            time_diffs = [
                (self.message_times[i] - self.message_times[i-1]) / 1e9
                for i in range(1, len(self.message_times))
            ]
            
            if time_diffs:
                avg_interval = statistics.mean(time_diffs)
                min_interval = min(time_diffs)
                max_interval = max(time_diffs)
                
                # Create diagnostic status
                diag_status = DiagnosticStatus()
                diag_status.name = 'topic_performance'
                diag_status.level = DiagnosticStatus.OK
                diag_status.message = 'Topic performance within acceptable range'
                
                # Add key-value pairs for detailed information
                diag_status.values.extend([
                    {'key': 'average_interval', 'value': f'{avg_interval:.3f}s'},
                    {'key': 'min_interval', 'value': f'{min_interval:.3f}s'},
                    {'key': 'max_interval', 'value': f'{max_interval:.3f}s'},
                    {'key': 'message_rate', 'value': f'{1.0/avg_interval:.2f} Hz'} if avg_interval > 0 else {'key': 'message_rate', 'value': '0 Hz'}
                ])
                
                diag_array.status.append(diag_status)
        
        self.diag_pub.publish(diag_array)

def main(args=None):
    rclpy.init(args=args)
    node = TopicMonitorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Monitor node interrupted')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Topic Best Practices

1. **Appropriate QoS Configuration**: Select QoS policies based on the data characteristics and requirements
2. **Topic Naming Conventions**: Use clear, consistent naming that indicates the data type and origin
3. **Message Type Selection**: Choose appropriate message types for your data, considering the balance between expressiveness and overhead
4. **Efficient Message Publishing**: Avoid excessive publishing that can overwhelm the system
5. **Memory Management**: Be mindful of message allocation and garbage collection in real-time systems

## ROS 2 Services

### Understanding Services in Depth

Services in ROS 2 implement a synchronous request-response communication pattern. Unlike topics, which provide one-way, asynchronous communication, services establish a direct, synchronous interaction between a service client and a service server. This pattern is essential for operations that require confirmation of completion, error handling, or immediate results.

The service pattern consists of three main components:
1. **Service Interface**: Defines the request and response message types
2. **Service Server**: Handles incoming requests and sends responses
3. **Service Client**: Sends requests and receives responses

Services are ideal for operations like:
- Configuration changes
- Triggering specific actions
- Querying system status
- Performing operations that must complete successfully

### Service Implementation and Architecture

Service implementation in ROS 2 follows a similar layered architecture to topics, but with different middleware considerations. Services typically use reliable communication patterns and may involve more complex state management than topics.

Let's explore comprehensive service implementations:

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool, Trigger
from std_srvs.srv import Empty
from rclpy.qos import qos_profile_services_default

class ComprehensiveServiceNode(Node):
    """
    Example node demonstrating various service patterns
    """
    
    def __init__(self):
        super().__init__('comprehensive_service_node')
        
        # Set up various services
        self.toggle_service = self.create_service(
            SetBool, 
            'toggle_robot', 
            self.toggle_robot_callback
        )
        
        self.trigger_service = self.create_service(
            Trigger,
            'robot_trigger',
            self.trigger_callback
        )
        
        self.config_service = self.create_service(
            SetBool,  # Using SetBool for demonstration; in practice, you'd define a custom .srv
            'robot_config',
            self.config_callback
        )
        
        # Keep track of robot state
        self.robot_enabled = True
        self.robot_triggers = 0
        
        self.get_logger().info('Comprehensive Service Node initialized')
    
    def toggle_robot_callback(self, request, response):
        """Toggle robot enable/disable state"""
        old_state = self.robot_enabled
        self.robot_enabled = request.data
        
        response.success = True
        if old_state != self.robot_enabled:
            response.message = f'Robot state changed: {old_state} -> {self.robot_enabled}'
        else:
            response.message = f'Robot state unchanged: {self.robot_enabled}'
        
        self.get_logger().info(f'Robot toggle: {response.message}')
        return response
    
    def trigger_callback(self, request, response):
        """Handle a robot trigger event"""
        self.robot_triggers += 1
        response.success = True
        response.message = f'Trigger event #{self.robot_triggers} executed'
        
        self.get_logger().info(f'Trigger event: {response.message}')
        return response
    
    def config_callback(self, request, response):
        """Handle configuration changes"""
        # In a real system, this might change robot parameters
        response.success = True
        response.message = f'Configuration updated to: {request.data}'
        
        self.get_logger().info(f'Configuration change: {response.message}')
        return response

class ServiceClientNode(Node):
    """
    Client node that uses the services provided by ComprehensiveServiceNode
    """
    
    def __init__(self):
        super().__init__('service_client_node')
        
        # Create clients for the services
        self.toggle_client = self.create_client(SetBool, 'toggle_robot')
        self.trigger_client = self.create_client(Trigger, 'robot_trigger')
        self.config_client = self.create_client(SetBool, 'robot_config')
        
        # Wait for services to be available
        while not self.toggle_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Toggle service not available, waiting again...')
        
        while not self.trigger_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Trigger service not available, waiting again...')
        
        while not self.config_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Config service not available, waiting again...')
        
        # Create timer to make service calls
        self.timer = self.create_timer(2.0, self.call_services)
        self.call_count = 0
        
        self.get_logger().info('Service Client Node initialized')
    
    def call_services(self):
        """Make service calls periodically"""
        if self.call_count % 3 == 0:
            # Toggle robot
            request = SetBool.Request()
            request.data = self.call_count % 6 == 0  # Toggle every 6 calls
            future = self.toggle_client.call_async(request)
            future.add_done_callback(self.toggle_callback)
        elif self.call_count % 3 == 1:
            # Trigger robot
            request = Trigger.Request()
            future = self.trigger_client.call_async(request)
            future.add_done_callback(self.trigger_callback)
        else:
            # Configure robot
            request = SetBool.Request()
            request.data = True
            future = self.config_client.call_async(request)
            future.add_done_callback(self.config_callback)
        
        self.call_count += 1
    
    def toggle_callback(self, future):
        """Handle toggle service response"""
        try:
            response = future.result()
            self.get_logger().info(f'Toggle response: {response.success}, {response.message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
    
    def trigger_callback(self, future):
        """Handle trigger service response"""
        try:
            response = future.result()
            self.get_logger().info(f'Trigger response: {response.success}, {response.message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
    
    def config_callback(self, future):
        """Handle config service response"""
        try:
            response = future.result()
            self.get_logger().info(f'Config response: {response.success}, {response.message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    
    # Create both nodes in the same process for demonstration
    service_node = ComprehensiveServiceNode()
    client_node = ServiceClientNode()
    
    # Create executor with both nodes
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(service_node)
    executor.add_node(client_node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        service_node.get_logger().info('Shutting down service nodes')
    finally:
        executor.shutdown()
        service_node.destroy_node()
        client_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Advanced Service Patterns

#### Asynchronous Service Calls
For better performance and responsiveness, services can be called asynchronously:

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool
import asyncio
from concurrent.futures import ThreadPoolExecutor

class AsyncServiceClientNode(Node):
    """
    Client node that makes asynchronous service calls
    """
    
    def __init__(self):
        super().__init__('async_service_client_node')
        
        # Create service client
        self.service_client = self.create_client(SetBool, 'async_service')
        
        # Wait for service to be available
        while not self.service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        
        # Set up async execution
        self.executor = ThreadPoolExecutor(max_workers=4)
        self.call_count = 0
        
        # Timer to initiate service calls
        self.timer = self.create_timer(0.5, self.make_async_calls)
        
        self.get_logger().info('Async Service Client Node initialized')
    
    def make_async_calls(self):
        """Initiate asynchronous service calls"""
        request = SetBool.Request()
        request.data = self.call_count % 2 == 0  # Alternate true/false
        
        # Make asynchronous call using executor
        future = self.service_client.call_async(request)
        future.add_done_callback(self.service_done_callback)
        
        self.call_count += 1
    
    def service_done_callback(self, future):
        """Handle completed service calls"""
        try:
            response = future.result()
            self.get_logger().info(
                f'Async service call completed: {response.success}, {response.message}'
            )
        except Exception as e:
            self.get_logger().error(f'Async service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = AsyncServiceClientNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Async client interrupted')
    finally:
        node.executor.shutdown(wait=True)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Service with Context7 Integration
Integrating Context7 for dynamic documentation access in service implementations:

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool
import requests
import json
from typing import Dict, Any, Optional

class Context7ServiceNode(Node):
    """
    Service node with Context7 documentation integration
    """
    
    def __init__(self):
        super().__init__('context7_service_node')
        
        # Main service with documentation
        self.documentation_service = self.create_service(
            SetBool,
            'get_documentation',
            self.get_documentation_callback
        )
        
        # Configuration service with live documentation
        self.config_service = self.create_service(
            SetBool,
            'config_with_docs',
            self.config_with_docs_callback
        )
        
        # Service to update documentation cache
        self.update_docs_service = self.create_service(
            SetBool,
            'update_documentation_cache',
            self.update_documentation_cache_callback
        )
        
        # Cache for documentation
        self.doc_cache: Dict[str, Any] = {}
        
        self.get_logger().info('Context7 Service Node initialized')
    
    def get_documentation_callback(self, request, response):
        """Provide documentation for ROS 2 concepts"""
        try:
            # In a real implementation, this would connect to Context7 MCP server
            # For now, we'll demonstrate the pattern with mock data
            
            doc_topic = "rclpy.services" if request.data else "rclpy.topics"
            
            # Mock documentation response (would come from Context7 in real implementation)
            documentation = self._get_mock_documentation(doc_topic)
            
            response.success = True
            response.message = f"Documentation for {doc_topic} retrieved successfully"
            
            # Log the documentation for demonstration
            self.get_logger().info(f"Provided documentation for: {doc_topic}")
            
        except Exception as e:
            self.get_logger().error(f"Error retrieving documentation: {e}")
            response.success = False
            response.message = f"Error retrieving documentation: {str(e)}"
        
        return response
    
    def config_with_docs_callback(self, request, response):
        """Configuration service with documentation references"""
        try:
            # Apply configuration
            new_config_value = request.data
            self.get_logger().info(f"Applying configuration: {new_config_value}")
            
            # In a real system, you might want to document the configuration change
            # and provide relevant documentation to the caller
            
            # Access Context7 documentation for best practices
            config_docs = self._get_mock_documentation("ros2.configuration.best_practices")
            
            response.success = True
            response.message = f"Configuration applied successfully. See documentation for best practices."
            
            # Log configuration with documentation reference
            self.get_logger().info(f"Configuration applied. Docs: {config_docs['title']}")
            
        except Exception as e:
            self.get_logger().error(f"Configuration error: {e}")
            response.success = False
            response.message = f"Configuration failed: {str(e)}"
        
        return response
    
    def update_documentation_cache_callback(self, request, response):
        """Update the local documentation cache from Context7"""
        try:
            # In a real implementation, this would query the Context7 MCP server
            # to update the local documentation cache
            
            # Mock cache update
            self.doc_cache = {
                "last_updated": self.get_clock().now().seconds_nanoseconds(),
                "topics": self._get_mock_documentation("rclpy.topics"),
                "services": self._get_mock_documentation("rclpy.services"),
                "actions": self._get_mock_documentation("rclpy.actions")
            }
            
            response.success = True
            response.message = f"Documentation cache updated with {len(self.doc_cache)} entries"
            
            self.get_logger().info("Documentation cache updated successfully")
            
        except Exception as e:
            self.get_logger().error(f"Cache update error: {e}")
            response.success = False
            response.message = f"Cache update failed: {str(e)}"
        
        return response
    
    def _get_mock_documentation(self, topic: str) -> Dict[str, Any]:
        """
        Mock function that simulates Context7 documentation retrieval.
        In a real implementation, this would connect to the Context7 MCP server.
        """
        mock_docs = {
            "rclpy.services": {
                "title": "ROS 2 Services in rclpy",
                "description": "Services provide synchronous request-response communication",
                "best_practices": [
                    "Use appropriate timeout values",
                    "Handle service call failures gracefully",
                    "Provide clear error messages",
                    "Document expected request/response formats"
                ],
                "examples": [
                    "Configuration services",
                    "Trigger services", 
                    "Status query services"
                ],
                "performance_considerations": [
                    "Services block the calling thread",
                    "Consider async calls for non-blocking operations",
                    "Use services for operations that require immediate results"
                ]
            },
            "rclpy.topics": {
                "title": "ROS 2 Topics in rclpy",
                "description": "Topics provide asynchronous publish-subscribe communication",
                "best_practices": [
                    "Choose appropriate QoS policies",
                    "Use message filters for synchronization",
                    "Avoid publishing too frequently",
                    "Consider message size for bandwidth"
                ],
                "examples": [
                    "Sensor data streams",
                    "Robot state publishing",
                    "Control command topics"
                ],
                "performance_considerations": [
                    "Topics are generally more efficient for streaming data",
                    "Consider reliability vs. performance trade-offs",
                    "Use appropriate history depth for your use case"
                ]
            },
            "ros2.configuration.best_practices": {
                "title": "ROS 2 Configuration Best Practices",
                "description": "Best practices for configuring ROS 2 nodes and systems",
                "guidelines": [
                    "Use parameters for runtime configuration",
                    "Provide sensible default values",
                    "Validate parameter values",
                    "Document parameter meanings and ranges"
                ],
                "patterns": [
                    "Parameter declarations with descriptions",
                    "Dynamic parameter handling",
                    "Configuration file management"
                ]
            }
        }
        
        return mock_docs.get(topic, {
            "title": f"Documentation for {topic}",
            "description": "No specific documentation available",
            "content": "This would be populated from Context7 in a real implementation"
        })

def main(args=None):
    rclpy.init(args=args)
    node = Context7ServiceNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Context7 service node interrupted')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Service Best Practices

1. **Service Interface Design**: Design service interfaces to be clear, concise, and self-documenting
2. **Error Handling**: Always handle potential service call failures gracefully
3. **Timeout Management**: Set appropriate timeouts to prevent indefinite blocking
4. **Resource Management**: Properly manage resources in service callbacks
5. **Security Considerations**: Implement service-specific security measures when needed
6. **Performance Monitoring**: Monitor service response times and success rates

## Advanced Communication Patterns

### Complex Data Structures with ROS 2 Messages

Modern robotic systems often require complex data structures that go beyond simple primitive types. ROS 2 provides mechanisms to handle complex messages including nested messages, arrays, and custom message types:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header, String, Int32MultiArray
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import PointCloud2, Image
from std_msgs.msg import MultiArrayLayout, MultiArrayDimension

class ComplexMessageNode(Node):
    """
    Node demonstrating complex message handling
    """
    
    def __init__(self):
        super().__init__('complex_message_node')
        
        # Publisher for complex robot state
        self.robot_state_pub = self.create_publisher(
            Pose,  # Using Pose as a simple complex message
            'robot_complex_state',
            10
        )
        
        # Publisher for multi-dimensional data
        self.array_pub = self.create_publisher(
            Int32MultiArray,
            'sensor_data_array',
            10
        )
        
        # Subscriber for complex incoming data
        self.complex_sub = self.create_subscription(
            Pose,
            'incoming_complex_data',
            self.complex_data_callback,
            10
        )
        
        # Timer for periodic complex data publishing
        self.timer = self.create_timer(1.0, self.publish_complex_data)
        
        self.get_logger().info('Complex Message Node initialized')
    
    def publish_complex_data(self):
        """Publish complex data structures"""
        # Publish pose (position + orientation)
        pose_msg = Pose()
        pose_msg.position.x = 1.0
        pose_msg.position.y = 2.0
        pose_msg.position.z = 0.0
        pose_msg.orientation.w = 1.0
        
        self.robot_state_pub.publish(pose_msg)
        
        # Publish multi-dimensional array (e.g., sensor matrix)
        array_msg = Int32MultiArray()
        
        # Define layout for 3x4 matrix
        dim1 = MultiArrayDimension()
        dim1.label = "rows"
        dim1.size = 3
        dim1.stride = 3 * 4  # Total size
        
        dim2 = MultiArrayDimension()
        dim2.label = "cols" 
        dim2.size = 4
        dim2.stride = 4  # Size of each row
        
        array_msg.layout.dim = [dim1, dim2]
        array_msg.layout.data_offset = 0
        
        # Fill data (3x4 matrix)
        array_msg.data = [
            1, 2, 3, 4,
            5, 6, 7, 8,
            9, 10, 11, 12
        ]
        
        self.array_pub.publish(array_msg)
    
    def complex_data_callback(self, msg):
        """Handle complex incoming data"""
        self.get_logger().info(
            f'Received complex data: [{msg.position.x}, {msg.position.y}, {msg.position.z}] '
            f'orientation: [{msg.orientation.x}, {msg.orientation.y}, {msg.orientation.z}, {msg.orientation.w}]'
        )

def main(args=None):
    rclpy.init(args=args)
    node = ComplexMessageNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Complex message node interrupted')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Custom Message Types

Creating and using custom message types is essential for specialized robotic applications:

```python
# This would typically be defined in a .msg file in your package
# For example: msg/RobotStatus.msg
"""
# Custom robot status message
std_msgs/Header header
string robot_name
int32 battery_level
bool is_charging
geometry_msgs/Pose current_pose
float32[] sensor_readings
string[] active_modes
"""

# Usage in a node:
import rclpy
from rclpy.node import Node
# from your_package.msg import RobotStatus  # Custom message

class CustomMessageNode(Node):
    def __init__(self):
        super().__init__('custom_message_node')
        
        # In a real implementation, you would import your custom message:
        # self.status_pub = self.create_publisher(RobotStatus, 'robot_status', 10)
        
        # For this example, we'll show the structure
        self.get_logger().info('Custom Message Node initialized')
        
        # Simulate custom message creation
        self.timer = self.create_timer(2.0, self.simulate_custom_message)
    
    def simulate_custom_message(self):
        """Simulate creating and publishing a custom message"""
        # This is pseudocode showing the structure:
        # msg = RobotStatus()
        # msg.header.stamp = self.get_clock().now().to_msg()
        # msg.header.frame_id = "base_link"
        # msg.robot_name = "autonomous_humanoid"
        # msg.battery_level = 87
        # msg.is_charging = False
        # msg.current_pose = self.get_current_pose()  # Implement based on your robot
        # msg.sensor_readings = [1.2, 3.4, 5.6, 7.8]  # Example sensor values
        # msg.active_modes = ["navigation", "perception"]
        # 
        # self.status_pub.publish(msg)
        # self.get_logger().info(f'Published custom status: battery {msg.battery_level}%')
        
        self.get_logger().info('Simulating custom message creation')

def main(args=None):
    rclpy.init(args=args)
    node = CustomMessageNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Custom message node interrupted')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Quality of Service (QoS) in Depth

### Understanding QoS Policies

Quality of Service (QoS) policies in ROS 2 provide fine-grained control over communication behavior. These policies are crucial for robotics applications where different types of data have different requirements for reliability, latency, and resource usage.

The four main QoS settings are:

1. **Reliability**: Determines whether messages are delivered reliably or best-effort
2. **Durability**: Determines whether late-joining subscribers receive past messages
3. **History**: Determines how many messages are stored for delivery
4. **Depth**: The maximum number of samples to queue when history policy is KEEP_LAST

### QoS Implementation Examples

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import (
    QoSProfile, 
    ReliabilityPolicy, 
    DurabilityPolicy, 
    HistoryPolicy,
    LivelinessPolicy
)

class QoSExampleNode(Node):
    """
    Node demonstrating various QoS configurations
    """
    
    def __init__(self):
        super().__init__('qos_example_node')
        
        # QoS profiles for different use cases
        self.setup_qos_profiles()
        
        # Publishers with different QoS requirements
        self.critical_pub = self.create_publisher(String, 'critical_commands', self.critical_qos)
        self.sensor_pub = self.create_publisher(LaserScan, 'sensor_data', self.sensor_qos)
        self.status_pub = self.create_publisher(String, 'status_updates', self.status_qos)
        self.debug_pub = self.create_publisher(String, 'debug_info', self.debug_qos)
        
        # Subscribers matching the QoS of publishers
        self.critical_sub = self.create_subscription(
            String, 'critical_commands', self.critical_callback, self.critical_qos
        )
        self.sensor_sub = self.create_subscription(
            LaserScan, 'sensor_data', self.sensor_callback, self.sensor_qos
        )
        
        # Timer for periodic publications
        self.timer = self.create_timer(0.1, self.publish_data)
        self.counter = 0
        
        self.get_logger().info('QoS Example Node initialized')
    
    def setup_qos_profiles(self):
        """Define various QoS profiles for different use cases"""
        
        # For critical commands that must be delivered reliably
        self.critical_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST
        )
        
        # For sensor data where we want the latest values
        self.sensor_qos = QoSProfile(
            depth=5,  # Keep last 5 sensor readings
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST
        )
        
        # For status updates with reliable delivery
        self.status_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST
        )
        
        # For debug info where loss is acceptable
        self.debug_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST
        )
    
    def publish_data(self):
        """Publish data with appropriate QoS"""
        # Publish critical command
        cmd_msg = String()
        cmd_msg.data = f'Critical command {self.counter}'
        self.critical_pub.publish(cmd_msg)
        
        # Publish sensor data (simplified)
        sensor_msg = LaserScan()
        sensor_msg.ranges = [1.0, 1.1, 1.2]  # Simulated ranges
        sensor_msg.angle_min = -1.57
        sensor_msg.angle_max = 1.57
        sensor_msg.angle_increment = 0.1
        self.sensor_pub.publish(sensor_msg)
        
        # Publish status update
        status_msg = String()
        status_msg.data = f'Status at {self.counter}'
        self.status_pub.publish(status_msg)
        
        # Publish debug info
        debug_msg = String()
        debug_msg.data = f'Debug: {self.counter}'
        self.debug_pub.publish(debug_msg)
        
        self.counter += 1
    
    def critical_callback(self, msg):
        """Handle critical commands"""
        self.get_logger().info(f'Critical: {msg.data}')
    
    def sensor_callback(self, msg):
        """Handle sensor data"""
        self.get_logger().debug(f'Sensor data received: {len(msg.ranges)} readings')

def main(args=None):
    rclpy.init(args=args)
    node = QoSExampleNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('QoS example node interrupted')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Advanced QoS Features

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.duration import Duration

class AdvancedQoSNode(Node):
    """
    Node demonstrating advanced QoS features
    """
    
    def __init__(self):
        super().__init__('advanced_qos_node')
        
        # QoS with deadline and lifespan
        self.deadline_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            deadline=Duration(seconds=1),  # Message must be delivered within 1 second
            lifespan=Duration(seconds=5)   # Message is valid for 5 seconds
        )
        
        # QoS with liveliness
        self.liveliness_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            # Liveliness checking can be added here
        )
        
        # Publishers with advanced QoS
        self.deadline_pub = self.create_publisher(String, 'deadline_topic', self.deadline_qos)
        self.liveliness_pub = self.create_publisher(String, 'liveliness_topic', self.liveliness_qos)
        
        # Subscribers
        self.deadline_sub = self.create_subscription(
            String, 'deadline_topic', self.deadline_callback, self.deadline_qos
        )
        
        # Timer for publications
        self.timer = self.create_timer(0.5, self.publish_with_qos)
        
        self.get_logger().info('Advanced QoS Node initialized')
    
    def publish_with_qos(self):
        """Publish messages with advanced QoS constraints"""
        msg = String()
        msg.data = f'Message with timestamp: {self.get_clock().now().nanoseconds}'
        self.deadline_pub.publish(msg)
    
    def deadline_callback(self, msg):
        """Handle messages with deadline constraints"""
        self.get_logger().info(f'Deadline message: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = AdvancedQoSNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Advanced QoS node interrupted')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## ROS 2 Actions

### Understanding Actions

Actions in ROS 2 extend the service pattern to handle long-running operations that provide feedback during execution and can be canceled. Actions are ideal for tasks like robot navigation, manipulation, or any operation that:

- Takes a significant amount of time
- Provides feedback during execution
- Can be preempted or canceled
- Has a clear goal that can succeed or fail

### Action Implementation

```python
import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import time
from threading import Thread

# In a real implementation, you would define custom action interfaces
# For example: action/NavigateToPose.action
"""
# Define goal, result, and feedback messages
# Goal: geometry_msgs/Pose target_pose
# Result: bool reached
# Feedback: float32 distance_remaining
"""

class ActionServerNode(Node):
    """
    Example action server for a navigation task
    """
    
    def __init__(self):
        super().__init__('action_server_node')
        
        # Create action server with callbacks
        # In a real system, you would use the actual action type:
        # self._action_server = ActionServer(
        #     self,
        #     NavigateToPose,
        #     'navigate_to_pose',
        #     execute_callback=self.execute_callback,
        #     goal_callback=self.goal_callback,
        #     cancel_callback=self.cancel_callback,
        #     callback_group=ReentrantCallbackGroup()
        # )
        
        self.get_logger().info('Action Server Node initialized')
        
        # For demonstration, we'll simulate action server functionality
        self.goal_active = False
        self.current_goal = None
    
    def goal_callback(self, goal_request):
        """Accept or reject goal requests"""
        # Accept all goals for this example
        return GoalResponse.ACCEPT
    
    def cancel_callback(self, goal_handle):
        """Accept or reject cancellation requests"""
        # Accept all cancellation requests
        return CancelResponse.ACCEPT
    
    async def execute_callback(self, goal_handle):
        """Execute the goal - this is where the main action logic goes"""
        self.get_logger().info('Executing goal...')
        
        # Simulate action execution
        feedback_msg = NavigateToPose.Feedback()
        result = NavigateToPose.Result()
        
        # Simulate moving toward goal
        for i in range(10):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.reached = False
                self.get_logger().info('Goal canceled')
                return result
            
            # Update feedback
            feedback_msg.distance_remaining = 10.0 - i
            goal_handle.publish_feedback(feedback_msg)
            
            # Simulate work by sleeping
            time.sleep(1)
        
        # Mark goal as succeeded
        goal_handle.succeed()
        result.reached = True
        self.get_logger().info('Goal succeeded')
        
        return result

class ActionClientNode(Node):
    """
    Example action client
    """
    
    def __init__(self):
        super().__init__('action_client_node')
        
        # In a real system:
        # self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        self.get_logger().info('Action Client Node initialized')
        
        # Timer to send goals periodically
        self.timer = self.create_timer(5.0, self.send_goal)
    
    def send_goal(self):
        """Send a goal to the action server"""
        self.get_logger().info('Sending navigation goal...')
        
        # In a real implementation:
        # goal_msg = NavigateToPose.Goal()
        # goal_msg.target_pose = self.get_target_pose()  # Define target pose
        # 
        # self._action_client.wait_for_server()
        # future = self._action_client.send_goal_async(
        #     goal_msg,
        #     feedback_callback=self.feedback_callback
        # )
        # future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        """Handle goal response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        
        self.get_logger().info('Goal accepted')
        # Get result future
        # result_future = goal_handle.get_result_async()
        # result_future.add_done_callback(self.get_result_callback)
    
    def feedback_callback(self, feedback_msg):
        """Handle feedback from action server"""
        self.get_logger().info(
            f'Received feedback: {feedback_msg.feedback.distance_remaining}m remaining'
        )
    
    def get_result_callback(self, future):
        """Handle action result"""
        result = future.result().result
        self.get_logger().info(f'Action completed with result: {result.reached}')

def main(args=None):
    rclpy.init(args=args)
    
    # For this example, we'll just demonstrate the structure
    # since we don't have actual action definition files
    node = ActionServerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Action server interrupted')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Context7 Integration for Documentation

### Dynamic Documentation Access

The integration of Context7 with ROS 2 development enables dynamic access to up-to-date documentation and best practices. This system allows developers to query documentation in real-time, enhancing the development process with current, relevant information.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import asyncio
import requests
from typing import Dict, Any, Optional

class Context7IntegrationNode(Node):
    """
    Node demonstrating Context7 integration for dynamic documentation
    """
    
    def __init__(self):
        super().__init__('context7_integration_node')
        
        # Publisher for documentation requests
        self.doc_request_pub = self.create_publisher(String, 'context7_requests', 10)
        
        # Subscriber for documentation responses
        self.doc_response_sub = self.create_subscription(
            String, 'context7_responses', self.doc_response_callback, 10
        )
        
        # Timer for periodic documentation queries
        self.doc_timer = self.create_timer(10.0, self.query_documentation)
        
        # Track documentation queries
        self.query_count = 0
        
        self.get_logger().info('Context7 Integration Node initialized')
    
    def query_documentation(self):
        """
        Query Context7 for relevant documentation
        In a real implementation, this would connect to the Context7 MCP server
        """
        topics_to_query = [
            "rclpy.topics",
            "rclpy.services", 
            "ros2.qos.policies",
            "ros2.best_practices"
        ]
        
        current_topic = topics_to_query[self.query_count % len(topics_to_query)]
        
        # In a real implementation, you would make an MCP call to the Context7 server
        # For demonstration, we'll simulate the process
        self.get_logger().info(f'Querying Context7 for: {current_topic}')
        
        # This would be replaced with actual MCP client code
        documentation = self._get_documentation_from_context7(current_topic)
        
        if documentation:
            self.get_logger().info(f'Retrieved documentation for: {current_topic}')
            self.get_logger().info(f'Doc title: {documentation.get("title", "No title")}')
        else:
            self.get_logger().warn(f'Failed to retrieve documentation for: {current_topic}')
        
        self.query_count += 1
    
    def _get_documentation_from_context7(self, topic: str) -> Optional[Dict[str, Any]]:
        """
        Simulate retrieving documentation from Context7 server.
        In a real implementation, this would connect to the Context7 MCP server.
        """
        # Mock implementation - in reality, this would make an MCP call
        mock_docs = {
            "rclpy.topics": {
                "title": "ROS 2 Topics with rclpy",
                "version": "humble",
                "last_updated": "2024-12-14",
                "description": "Topics provide asynchronous publish-subscribe communication in ROS 2",
                "best_practices": [
                    "Use appropriate QoS profiles for your data type",
                    "Consider memory usage when publishing large messages",
                    "Use message filters for synchronization of multiple topics",
                    "Implement proper error handling in callbacks"
                ],
                "examples": [
                    "Sensor data streaming",
                    "Robot state publishing",
                    "Control command distribution"
                ],
                "performance_tips": [
                    "Consider using BEST_EFFORT for sensor data where occasional loss is acceptable",
                    "Use RELIABLE for critical control messages",
                    "Choose history depth based on your application needs"
                ],
                "common_pitfalls": [
                    "Publishing too frequently without considering network bandwidth",
                    "Not handling callback exceptions, which can crash the node",
                    "Incorrect QoS profile matching between publishers and subscribers"
                ]
            },
            "rclpy.services": {
                "title": "ROS 2 Services with rclpy",
                "version": "humble",
                "last_updated": "2024-12-14",
                "description": "Services provide synchronous request-response communication",
                "best_practices": [
                    "Use appropriate timeouts for service calls",
                    "Handle service call failures gracefully",
                    "Keep service operations as fast as possible",
                    "Provide clear error messages in service responses"
                ],
                "examples": [
                    "Configuration changes",
                    "Triggering specific actions",
                    "Querying system status"
                ],
                "performance_considerations": [
                    "Services block the calling thread until response",
                    "Consider async calls for non-blocking operations",
                    "Don't perform long-running operations in service callbacks"
                ]
            },
            "ros2.qos.policies": {
                "title": "ROS 2 Quality of Service Policies",
                "version": "humble",
                "last_updated": "2024-12-14",
                "description": "QoS policies control message delivery behavior in ROS 2",
                "policies": {
                    "Reliability": {
                        "RELIABLE": "Messages are guaranteed to be delivered, possibly multiple times",
                        "BEST_EFFORT": "Messages are sent without guarantee of delivery"
                    },
                    "Durability": {
                        "VOLATILE": "New subscribers don't receive messages published before they joined",
                        "TRANSIENT_LOCAL": "New subscribers receive messages published before they joined"
                    },
                    "History": {
                        "KEEP_LAST": "Only keep the specified number of most recent messages",
                        "KEEP_ALL": "Keep all messages (use with caution)"
                    }
                }
            },
            "ros2.best_practices": {
                "title": "ROS 2 Best Practices",
                "version": "humble",
                "last_updated": "2024-12-14",
                "categories": [
                    "Node Design",
                    "Communication Patterns",
                    "Error Handling",
                    "Performance Optimization"
                ],
                "practices": [
                    "Use meaningful node and topic names",
                    "Implement proper error handling",
                    "Choose appropriate QoS settings",
                    "Use parameters for runtime configuration",
                    "Follow the ROS 2 coding standards"
                ]
            }
        }
        
        return mock_docs.get(topic)
    
    def doc_response_callback(self, msg):
        """Handle documentation response from Context7"""
        self.get_logger().info(f'Documentation response: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = Context7IntegrationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Context7 integration node interrupted')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Context7 Documentation Caching System

For efficient access to frequently used documentation, a caching system can be implemented:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import hashlib
import json
from datetime import datetime, timedelta
from typing import Dict, Any, Optional

class Context7CacheNode(Node):
    """
    Node implementing a Context7 documentation caching system
    """
    
    def __init__(self):
        super().__init__('context7_cache_node')
        
        # Documentation cache with expiration
        self.doc_cache: Dict[str, Dict[str, Any]] = {}
        self.cache_ttl = timedelta(hours=1)  # Cache TTL: 1 hour
        
        # Timer for cache maintenance
        self.cache_timer = self.create_timer(300.0, self.maintain_cache)  # Every 5 minutes
        
        # Documentation access statistics
        self.access_stats = {}
        
        self.get_logger().info('Context7 Cache Node initialized')
    
    def get_documentation(self, topic: str) -> Optional[Dict[str, Any]]:
        """
        Retrieve documentation with caching
        """
        cache_key = self._generate_cache_key(topic)
        
        # Check if cached and not expired
        if cache_key in self.doc_cache:
            cached_item = self.doc_cache[cache_key]
            cached_time = datetime.fromisoformat(cached_item['timestamp'])
            
            if datetime.now() - cached_time < self.cache_ttl:
                # Return cached documentation
                self._update_access_stats(topic, 'cache_hit')
                self.get_logger().debug(f'Cache hit for: {topic}')
                return cached_item['documentation']
            else:
                # Cache expired, remove it
                del self.doc_cache[cache_key]
        
        # Fetch from Context7 (simulated)
        documentation = self._fetch_from_context7(topic)
        if documentation:
            # Cache the new documentation
            self.doc_cache[cache_key] = {
                'documentation': documentation,
                'timestamp': datetime.now().isoformat()
            }
            self._update_access_stats(topic, 'fetch_success')
            return documentation
        else:
            self._update_access_stats(topic, 'fetch_failure')
            return None
    
    def _generate_cache_key(self, topic: str) -> str:
        """Generate a cache key for the topic"""
        return hashlib.md5(topic.encode()).hexdigest()
    
    def _fetch_from_context7(self, topic: str) -> Optional[Dict[str, Any]]:
        """
        Simulate fetching documentation from Context7.
        In a real implementation, this would connect to the Context7 MCP server.
        """
        # Simulate a delay to mimic network request
        import time
        time.sleep(0.1)
        
        # Return mock documentation
        mock_docs = {
            "rclpy.node": {
                "title": "ROS 2 Nodes with rclpy",
                "content": "Nodes are the fundamental building blocks of ROS 2...",
                "last_updated": "2024-12-14",
                "version": "humble"
            },
            "rclpy.topics": {
                "title": "ROS 2 Topics with rclpy", 
                "content": "Topics enable asynchronous communication...",
                "last_updated": "2024-12-14",
                "version": "humble"
            },
            "rclpy.services": {
                "title": "ROS 2 Services with rclpy",
                "content": "Services provide synchronous request-response communication...",
                "last_updated": "2024-12-14", 
                "version": "humble"
            }
        }
        
        return mock_docs.get(topic)
    
    def _update_access_stats(self, topic: str, action: str):
        """Update access statistics"""
        if topic not in self.access_stats:
            self.access_stats[topic] = {'total': 0, 'cache_hit': 0, 'fetch_success': 0, 'fetch_failure': 0}
        
        self.access_stats[topic]['total'] += 1
        self.access_stats[topic][action] += 1
    
    def maintain_cache(self):
        """Periodically maintain the cache by removing expired entries"""
        current_time = datetime.now()
        expired_keys = []
        
        for key, cached_item in self.doc_cache.items():
            cached_time = datetime.fromisoformat(cached_item['timestamp'])
            if current_time - cached_time >= self.cache_ttl:
                expired_keys.append(key)
        
        for key in expired_keys:
            del self.doc_cache[key]
        
        self.get_logger().info(
            f'Cache maintenance: {len(expired_keys)} expired entries removed, '
            f'{len(self.doc_cache)} entries remaining'
        )
        
        # Log access statistics periodically
        if self.access_stats:
            self.get_logger().info('Documentation access statistics:')
            for topic, stats in self.access_stats.items():
                self.get_logger().info(f'  {topic}: {stats}')

def main(args=None):
    rclpy.init(args=args)
    node = Context7CacheNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Context7 cache node interrupted')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Practical Examples

### Robot Navigation System with Topics and Services

Here's a comprehensive example of a robot navigation system that demonstrates the practical use of topics and services:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, Point
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from example_interfaces.srv import Trigger, SetBool
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import math

class NavigationSystemNode(Node):
    """
    Comprehensive navigation system demonstrating topics and services
    """
    
    def __init__(self):
        super().__init__('navigation_system_node')
        
        # QoS profiles
        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )
        
        cmd_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST
        )
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', cmd_qos)
        self.status_pub = self.create_publisher(String, 'navigation_status', 10)
        
        # Subscribers
        self.laser_sub = self.create_subscription(
            LaserScan, 'scan', self.laser_callback, sensor_qos
        )
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, 'amcl_pose', self.pose_callback, 10
        )
        
        # Services
        self.nav_start_srv = self.create_service(
            Trigger, 'start_navigation', self.start_navigation_callback
        )
        self.nav_stop_srv = self.create_service(
            Trigger, 'stop_navigation', self.stop_navigation_callback
        )
        self.nav_pause_srv = self.create_service(
            SetBool, 'pause_navigation', self.pause_navigation_callback
        )
        
        # Navigation state
        self.navigation_active = False
        self.navigation_paused = False
        self.target_pose = None
        self.current_pose = None
        self.obstacle_detected = False
        self.min_obstacle_distance = float('inf')
        
        # Control timer
        self.control_timer = self.create_timer(0.1, self.navigation_control_loop)
        
        self.get_logger().info('Navigation System Node initialized')
    
    def laser_callback(self, msg):
        """Handle laser scan data for obstacle detection"""
        if len(msg.ranges) > 0:
            # Find minimum distance to obstacle
            valid_ranges = [r for r in msg.ranges if not math.isnan(r) and r > 0]
            if valid_ranges:
                self.min_obstacle_distance = min(valid_ranges)
                self.obstacle_detected = self.min_obstacle_distance < 0.5  # 0.5m threshold
            else:
                self.min_obstacle_distance = float('inf')
                self.obstacle_detected = False
        else:
            self.obstacle_detected = False
    
    def pose_callback(self, msg):
        """Handle pose updates"""
        self.current_pose = msg.pose.pose
    
    def start_navigation_callback(self, request, response):
        """Start navigation service"""
        if not self.navigation_active:
            self.navigation_active = True
            self.navigation_paused = False
            response.success = True
            response.message = 'Navigation started'
            self.get_logger().info('Navigation started')
        else:
            response.success = False
            response.message = 'Navigation already active'
        
        return response
    
    def stop_navigation_callback(self, request, response):
        """Stop navigation service"""
        if self.navigation_active:
            self.navigation_active = False
            self.navigation_paused = False
            self._stop_robot()
            response.success = True
            response.message = 'Navigation stopped'
            self.get_logger().info('Navigation stopped')
        else:
            response.success = False
            response.message = 'Navigation not active'
        
        return response
    
    def pause_navigation_callback(self, request, response):
        """Pause/resume navigation service"""
        self.navigation_paused = request.data
        response.success = True
        response.message = f'Navigation {"paused" if self.navigation_paused else "resumed"}'
        self.get_logger().info(f'Navigation {response.message}')
        
        if self.navigation_paused:
            self._stop_robot()
        
        return response
    
    def navigation_control_loop(self):
        """Main navigation control loop"""
        if not self.navigation_active or self.navigation_paused:
            self._publish_status()
            return
        
        cmd_vel = Twist()
        
        if self.obstacle_detected:
            # Emergency stop if obstacle too close
            if self.min_obstacle_distance < 0.3:  # 0.3m threshold
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = 0.0
                self.get_logger().warn(f'Obstacle too close: {self.min_obstacle_distance:.2f}m')
            else:
                # Turn away from obstacle
                cmd_vel.angular.z = 0.5  # Rotate to avoid
        else:
            # Proceed with navigation logic (simplified)
            cmd_vel.linear.x = 0.2  # Move forward
            cmd_vel.angular.z = 0.0  # No rotation
        
        # Publish command
        self.cmd_vel_pub.publish(cmd_vel)
        self._publish_status()
    
    def _stop_robot(self):
        """Stop the robot by publishing zero velocity"""
        stop_cmd = Twist()
        stop_cmd.linear.x = 0.0
        stop_cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(stop_cmd)
    
    def _publish_status(self):
        """Publish navigation status"""
        status_msg = String()
        if not self.navigation_active:
            status_msg.data = 'idle'
        elif self.navigation_paused:
            status_msg.data = 'paused'
        elif self.obstacle_detected:
            status_msg.data = f'obstacle_detected_{self.min_obstacle_distance:.2f}m'
        else:
            status_msg.data = 'navigating'
        
        self.status_pub.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    node = NavigationSystemNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Navigation system interrupted')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Sensor Fusion System

A sensor fusion system demonstrates the coordination of multiple topics and services:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu, NavSatFix
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import statistics
from collections import deque

class SensorFusionNode(Node):
    """
    Advanced sensor fusion system combining multiple sensor inputs
    """
    
    def __init__(self):
        super().__init__('sensor_fusion_node')
        
        # QoS profiles for different sensor types
        high_freq_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )
        
        reliable_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST
        )
        
        # Sensor subscribers
        self.laser_sub = self.create_subscription(
            LaserScan, 'scan', self.laser_callback, high_freq_qos
        )
        self.imu_sub = self.create_subscription(
            Imu, 'imu/data', self.imu_callback, high_freq_qos
        )
        self.gps_sub = self.create_subscription(
            NavSatFix, 'gps/fix', self.gps_callback, reliable_qos
        )
        
        # Publishers
        self.fused_data_pub = self.create_publisher(String, 'fused_sensor_data', 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Sensor data storage
        self.laser_ranges = deque(maxlen=100)
        self.imu_readings = deque(maxlen=100)
        self.gps_readings = deque(maxlen=50)
        
        # Fusion timer
        self.fusion_timer = self.create_timer(0.1, self.fusion_loop)
        
        self.get_logger().info('Sensor Fusion Node initialized')
    
    def laser_callback(self, msg):
        """Handle laser scan data"""
        # Only store finite, valid range values
        valid_ranges = [r for r in msg.ranges if not (math.isnan(r) or math.isinf(r)) and r > 0]
        self.laser_ranges.extend(valid_ranges[:10])  # Store first 10 readings as sample
    
    def imu_callback(self, msg):
        """Handle IMU data"""
        imu_data = {
            'orientation': [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w],
            'angular_velocity': [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z],
            'linear_acceleration': [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z],
            'timestamp': self.get_clock().now().nanoseconds
        }
        self.imu_readings.append(imu_data)
    
    def gps_callback(self, msg):
        """Handle GPS data"""
        gps_data = {
            'latitude': msg.latitude,
            'longitude': msg.longitude,
            'altitude': msg.altitude,
            'position_covariance': list(msg.position_covariance),
            'timestamp': self.get_clock().now().nanoseconds
        }
        self.gps_readings.append(gps_data)
    
    def fusion_loop(self):
        """Main fusion loop - combines sensor data to produce fused output"""
        fused_data = {
            'timestamp': self.get_clock().now().nanoseconds,
            'sensor_count': {
                'laser': len(self.laser_ranges),
                'imu': len(self.imu_readings),
                'gps': len(self.gps_readings)
            }
        }
        
        # Analyze laser data for obstacles
        if self.laser_ranges:
            avg_distance = statistics.mean(self.laser_ranges) if self.laser_ranges else float('inf')
            min_distance = min(self.laser_ranges) if self.laser_ranges else float('inf')
            fused_data['obstacle_analysis'] = {
                'average_distance': avg_distance,
                'min_distance': min_distance,
                'obstacle_warning': min_distance < 1.0  # Warning if closer than 1m
            }
        
        # Analyze IMU data for motion
        if self.imu_readings:
            latest_imu = self.imu_readings[-1]
            fused_data['motion_analysis'] = {
                'orientation': latest_imu['orientation'],
                'linear_acceleration_magnitude': math.sqrt(
                    latest_imu['linear_acceleration'][0]**2 +
                    latest_imu['linear_acceleration'][1]**2 +
                    latest_imu['linear_acceleration'][2]**2
                )
            }
        
        # Analyze GPS data for position
        if self.gps_readings:
            latest_gps = self.gps_readings[-1]
            fused_data['position_analysis'] = {
                'latitude': latest_gps['latitude'],
                'longitude': latest_gps['longitude'],
                'altitude': latest_gps['altitude']
            }
        
        # Publish fused data
        fused_msg = String()
        fused_msg.data = str(fused_data)
        self.fused_data_pub.publish(fused_msg)
        
        # Generate control commands based on fused data
        self._generate_control_commands(fused_data)
    
    def _generate_control_commands(self, fused_data):
        """Generate control commands based on fused sensor data"""
        cmd = Twist()
        
        # Simple obstacle avoidance based on fused data
        if 'obstacle_analysis' in fused_data:
            if fused_data['obstacle_analysis']['obstacle_warning']:
                # Emergency stop if obstacle too close
                cmd.linear.x = 0.0
                cmd.angular.z = 0.5  # Turn to avoid
            else:
                # Normal navigation
                cmd.linear.x = 0.2
                cmd.angular.z = 0.0
        
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = SensorFusionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Sensor fusion node interrupted')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Best Practices and Troubleshooting

### Performance Optimization

1. **QoS Selection**: Choose appropriate QoS settings for your data type
2. **Message Size**: Minimize message sizes to reduce network overhead
3. **Callback Efficiency**: Keep callbacks lightweight and avoid blocking operations
4. **Resource Management**: Properly manage publishers, subscribers, and services
5. **Threading**: Use appropriate threading models for your application

### Memory Management

ROS 2 provides mechanisms to help with memory management in real-time systems:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile
import gc

class MemoryEfficientNode(Node):
    """
    Example of memory-efficient node implementation
    """
    
    def __init__(self):
        super().__init__('memory_efficient_node')
        
        # Publishers with conservative QoS for memory usage
        self.publisher = self.create_publisher(
            String, 
            'memory_efficient_topic', 
            QoSProfile(depth=1)  # Minimal history to save memory
        )
        
        # Pre-allocate message objects to avoid frequent allocation
        self.msg_buffer = String()
        
        self.timer = self.create_timer(0.1, self.publish_efficiently)
        
        # Memory monitoring
        self.publish_count = 0
        self.memory_monitor_timer = self.create_timer(5.0, self.monitor_memory)
        
        self.get_logger().info('Memory Efficient Node initialized')
    
    def publish_efficiently(self):
        """Publish using pre-allocated message and efficient patterns"""
        # Reuse the same message object
        self.msg_buffer.data = f'Memory efficient message {self.publish_count}'
        self.publisher.publish(self.msg_buffer)
        self.publish_count += 1
        
        # Periodically trigger garbage collection if needed
        if self.publish_count % 100 == 0:
            collected = gc.collect()
            self.get_logger().debug(f'Garbage collected: {collected} objects')
    
    def monitor_memory(self):
        """Monitor memory usage patterns"""
        # In a real implementation, you might integrate with memory monitoring tools
        self.get_logger().info(
            f'Memory monitoring: Published {self.publish_count} messages, '
            f'current node memory usage: [would require platform-specific implementation]'
        )

def main(args=None):
    rclpy.init(args=args)
    node = MemoryEfficientNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Memory efficient node interrupted')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Common Troubleshooting Steps

1. **Topic Discovery**: Use `ros2 topic list` and `ros2 topic info` to diagnose communication issues
2. **Service Discovery**: Use `ros2 service list` and `ros2 service info` to check service availability
3. **QoS Matching**: Ensure publishers and subscribers use compatible QoS profiles
4. **Network Configuration**: Check DDS configuration for multi-machine communication
5. **Memory Leaks**: Monitor for proper destruction of publishers, subscribers, and services

### Debugging Complex Communication Issues

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
from collections import defaultdict, deque
import time

class CommunicationDebuggerNode(Node):
    """
    Node to debug and diagnose communication issues
    """
    
    def __init__(self):
        super().__init__('communication_debugger_node')
        
        # Track message statistics
        self.message_stats = defaultdict(lambda: {
            'received': defaultdict(int),
            'published': defaultdict(int),
            'timing': deque(maxlen=100)
        })
        
        # Debug publishers and subscribers
        self.debug_pub = self.create_publisher(String, 'debug_topic', 10)
        self.debug_sub = self.create_subscription(
            String, 'debug_topic', self.debug_callback, 10
        )
        
        # Timer for periodic diagnostics
        self.diagnostic_timer = self.create_timer(2.0, self.run_diagnostics)
        
        self.msg_counter = 0
        self.start_time = time.time()
        
        self.get_logger().info('Communication Debugger Node initialized')
    
    def debug_callback(self, msg):
        """Debug message callback with detailed logging"""
        current_time = time.time()
        self.message_stats['debug_topic']['received'][self.get_clock().now().nanoseconds] += 1
        
        # Store timing information
        self.message_stats['debug_topic']['timing'].append(current_time)
        
        self.get_logger().debug(f'Debug message received: {msg.data}')
    
    def run_diagnostics(self):
        """Run communication diagnostics"""
        elapsed = time.time() - self.start_time
        
        # Publish test message
        test_msg = String()
        test_msg.data = f'Debug message {self.msg_counter} at {elapsed:.2f}s'
        self.debug_pub.publish(test_msg)
        self.msg_counter += 1
        
        # Analyze statistics
        debug_stats = self.message_stats['debug_topic']
        received_count = len(debug_stats['received'])
        
        if self.msg_counter > 0:
            success_rate = (received_count / self.msg_counter) * 100
            self.get_logger().info(
                f'Diagnostics: {received_count}/{self.msg_counter} messages '
                f'({success_rate:.1f}% success rate)'
            )
            
            if received_count > 1 and len(debug_stats['timing']) > 1:
                # Calculate average message interval
                intervals = [
                    debug_stats['timing'][i+1] - debug_stats['timing'][i]
                    for i in range(len(debug_stats['timing']) - 1)
                ]
                if intervals:
                    avg_interval = sum(intervals) / len(intervals)
                    self.get_logger().info(f'Average receive interval: {avg_interval:.3f}s')
    
    def diagnostic_summary(self):
        """Print comprehensive diagnostic summary"""
        summary = "Communication Diagnostic Summary:\n"
        for topic, stats in self.message_stats.items():
            summary += f"  Topic {topic}:\n"
            summary += f"    Messages received: {len(stats['received'])}\n"
            summary += f"    Timing samples: {len(stats['timing'])}\n"
            if stats['timing']:
                avg_interval = sum(
                    stats['timing'][i+1] - stats['timing'][i] 
                    for i in range(len(stats['timing']) - 1)
                ) / max(1, len(stats['timing']) - 1)
                summary += f"    Average interval: {avg_interval:.3f}s\n"
        
        self.get_logger().info(summary)

def main(args=None):
    rclpy.init(args=args)
    node = CommunicationDebuggerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.diagnostic_summary()
        node.get_logger().info('Communication debugger interrupted')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Real-World Applications

### Industrial Robotics Communication Pattern

In industrial robotics, communication patterns must be extremely reliable and deterministic:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.time import Duration
import threading

class IndustrialRobotControllerNode(Node):
    """
    Industrial robot controller with safety and reliability features
    """
    
    def __init__(self):
        super().__init__('industrial_robot_controller_node')
        
        # High-reliability QoS for safety-critical applications
        critical_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST
        )
        
        # Publishers
        self.joint_trajectory_pub = self.create_publisher(
            JointTrajectory, 'joint_trajectory', critical_qos
        )
        self.robot_state_pub = self.create_publisher(
            JointTrajectoryControllerState, 'robot_state', critical_qos
        )
        
        # Subscribers with critical QoS
        self.joint_state_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, critical_qos
        )
        self.emergency_stop_sub = self.create_subscription(
            String, 'emergency_stop', self.emergency_stop_callback, critical_qos
        )
        
        # Safety monitoring
        self.joint_states = {}
        self.emergency_stop_active = False
        self.last_state_update = self.get_clock().now()
        
        # Control loop
        self.control_timer = self.create_timer(0.01, self.control_loop)  # 100Hz for industrial
        
        # Safety checks
        self.safety_timer = self.create_timer(0.1, self.safety_check)
        
        self.get_logger().info('Industrial Robot Controller Node initialized')
    
    def joint_state_callback(self, msg):
        """Update joint states with safety monitoring"""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.joint_states[name] = {
                    'position': msg.position[i],
                    'velocity': msg.velocity[i] if i < len(msg.velocity) else 0.0,
                    'effort': msg.effort[i] if i < len(msg.effort) else 0.0,
                    'timestamp': self.get_clock().now()
                }
        
        self.last_state_update = self.get_clock().now()
    
    def emergency_stop_callback(self, msg):
        """Handle emergency stop requests"""
        if msg.data == "EMERGENCY_STOP":
            self.emergency_stop_active = True
            self._execute_emergency_stop()
        elif msg.data == "RESET_EMERGENCY":
            self.emergency_stop_active = False
            self.get_logger().warn('Emergency stop reset - resume with caution')
    
    def control_loop(self):
        """Main control loop with safety checks"""
        if self.emergency_stop_active:
            self._publish_idle_position()
            return
        
        # Implement robot control logic here
        # For demo, just publish current state
        state_msg = JointTrajectoryControllerState()
        # Populate with actual robot state
        state_msg.header.stamp = self.get_clock().now().to_msg()
        self.robot_state_pub.publish(state_msg)
    
    def safety_check(self):
        """Periodic safety checks"""
        current_time = self.get_clock().now()
        
        # Check for communication timeouts
        if (current_time - self.last_state_update).nanoseconds > 1e9:  # 1 second
            self.get_logger().error('Joint state timeout - emergency stop')
            self.emergency_stop_active = True
            self._execute_emergency_stop()
    
    def _execute_emergency_stop(self):
        """Execute emergency stop sequence"""
        self.get_logger().fatal('EMERGENCY STOP ACTIVATED')
        
        # Send stop trajectory
        stop_trajectory = JointTrajectory()
        stop_point = JointTrajectoryPoint()
        stop_point.time_from_start = Duration(seconds=0.1).to_msg()
        # Set all joint velocities to zero for immediate stop
        stop_trajectory.points = [stop_point]
        
        self.joint_trajectory_pub.publish(stop_trajectory)
    
    def _publish_idle_position(self):
        """Publish safe idle position when robot is stopped"""
        idle_trajectory = JointTrajectory()
        idle_point = JointTrajectoryPoint()
        idle_point.time_from_start = Duration(seconds=0.1).to_msg()
        # Set safe joint positions
        idle_trajectory.points = [idle_point]
        
        self.joint_trajectory_pub.publish(idle_trajectory)

def main(args=None):
    rclpy.init(args=args)
    node = IndustrialRobotControllerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Industrial robot controller interrupted')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Multi-Robot Communication Architecture

For multi-robot systems, additional considerations for coordination and communication are required:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.executors import MultiThreadedExecutor
import json
import uuid

class MultiRobotCoordinatorNode(Node):
    """
    Node that coordinates multiple robots in a team
    """
    
    def __init__(self):
        super().__init__('multi_robot_coordinator_node')
        
        # QoS for multi-robot communication
        coordinator_qos = QoSProfile(
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_ALL
        )
        
        # Publishers for commands to robots
        self.command_pubs = {}
        for robot_id in ['robot1', 'robot2', 'robot3']:
            self.command_pubs[robot_id] = self.create_publisher(
                String, f'{robot_id}/command', coordinator_qos
            )
        
        # Subscribers for robot status
        self.status_subs = {}
        for robot_id in ['robot1', 'robot2', 'robot3']:
            self.status_subs[robot_id] = self.create_subscription(
                String, f'{robot_id}/status', 
                lambda msg, id=robot_id: self.robot_status_callback(msg, id), 
                coordinator_qos
            )
        
        # Team coordination topics
        self.task_assignment_pub = self.create_publisher(String, 'task_assignment', coordinator_qos)
        self.team_status_pub = self.create_publisher(String, 'team_status', coordinator_qos)
        
        # Store robot states
        self.robot_states = {
            'robot1': {'status': 'idle', 'position': None, 'task': None},
            'robot2': {'status': 'idle', 'position': None, 'task': None},
            'robot3': {'status': 'idle', 'position': None, 'task': None}
        }
        
        # Coordination timer
        self.coordination_timer = self.create_timer(1.0, self.coordinate_robots)
        
        self.get_logger().info('Multi-Robot Coordinator Node initialized')
    
    def robot_status_callback(self, msg, robot_id):
        """Handle status updates from individual robots"""
        try:
            status_data = json.loads(msg.data)
            self.robot_states[robot_id].update(status_data)
            self.get_logger().debug(f'Robot {robot_id} status: {status_data}')
        except json.JSONDecodeError:
            self.get_logger().warn(f'Invalid status message from {robot_id}: {msg.data}')
    
    def coordinate_robots(self):
        """Main coordination logic"""
        # Simple task assignment - in real applications, this would be more complex
        available_robots = [
            robot_id for robot_id, state in self.robot_states.items()
            if state['status'] == 'idle' and state['task'] is None
        ]
        
        if available_robots:
            # Assign a simple task to an available robot
            robot_to_assign = available_robots[0]
            task_assignment = {
                'task_id': str(uuid.uuid4()),
                'task_type': 'explore',
                'target_position': {'x': 1.0, 'y': 2.0, 'z': 0.0},
                'assigned_robot': robot_to_assign
            }
            
            # Send assignment to robot
            assignment_msg = String()
            assignment_msg.data = json.dumps(task_assignment)
            self.command_pubs[robot_to_assign].publish(assignment_msg)
            
            # Update robot state
            self.robot_states[robot_to_assign]['task'] = task_assignment['task_id']
            self.robot_states[robot_to_assign]['status'] = 'executing'
            
            # Publish to team task assignment
            self.task_assignment_pub.publish(assignment_msg)
            
            self.get_logger().info(f'Assigned task to {robot_to_assign}')
        
        # Publish overall team status
        team_status_msg = String()
        team_status_msg.data = json.dumps({
            'timestamp': self.get_clock().now().nanoseconds,
            'robot_states': self.robot_states
        })
        self.team_status_pub.publish(team_status_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MultiRobotCoordinatorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Multi-robot coordinator interrupted')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Future Developments

### Emerging Trends in ROS 2 Communication

ROS 2 communication continues to evolve with new developments in 2025 and beyond:

1. **Real-time Enhancements**: Improved real-time performance with deterministic behavior
2. **Security Improvements**: Enhanced security features for safety-critical applications
3. **Cloud Integration**: Better integration with cloud services and edge computing
4. **AI/ML Integration**: Native support for AI/ML model deployment and inference
5. **5G and Edge Computing**: Leverage high-speed, low-latency communication networks

### Performance Optimization Techniques

Recent developments in ROS 2 communication include advanced performance optimization techniques:

1. **Shared Memory Communication**: Direct memory sharing between nodes on the same machine
2. **Custom Middleware Implementations**: Specialized middleware for specific use cases
3. **Hardware Acceleration**: Direct integration with GPUs and other accelerators
4. **Predictive QoS**: AI-driven QoS policy selection based on network conditions

### Integration with Modern Technologies

ROS 2 continues to integrate with modern technologies:

1. **WebRTC for Robotics**: Real-time communication over web protocols
2. **gRPC Integration**: Integration with gRPC for cross-platform communication
3. **MessagePack**: More efficient serialization formats
4. **Time-Sensitive Networking (TSN)**: Deterministic networking for industrial applications

## Summary

ROS 2 Topics and Services form the foundation of modern robotic communication systems. The publish-subscribe model of topics provides asynchronous, one-way communication ideal for streaming sensor data and robot states. The request-response model of services provides synchronous communication perfect for configuration changes, triggering actions, and querying system status.

The introduction of DDS middleware in ROS 2 has revolutionized robotic communication by providing decentralized architecture, native security, and configurable Quality of Service policies. These improvements make ROS 2 suitable for safety-critical applications where reliability and performance are paramount.

Context7 integration enhances the development experience by providing access to up-to-date documentation and best practices through the Model Context Protocol. This integration allows developers to access relevant information during development without leaving their development environment.

For effective ROS 2 communication development:

1. Always consider the appropriate QoS settings for your specific use case
2. Use topics for streaming data and services for synchronous operations
3. Implement proper error handling and safety measures
4. Optimize for performance considering your hardware constraints
5. Integrate documentation access patterns for maintainable code
6. Follow security best practices for connected robotic systems

The robotic nervous system that ROS 2 provides enables complex, distributed robotic applications with reliable, efficient communication between all system components. As robotics continues to advance, these communication patterns will continue to evolve to meet the increasing demands of more sophisticated robotic systems.

Through careful design and implementation of topics and services, developers can create robust, scalable robotic applications that leverage the full power of ROS 2's distributed architecture. The combination of modern communication patterns with Context7 documentation integration creates a powerful foundation for next-generation robotic development.