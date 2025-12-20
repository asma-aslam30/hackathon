---
sidebar_position: 4
sidebar_label: Advanced Communication Patterns and Bridging
---

# Advanced Communication Patterns and Bridging in ROS 2

## Introduction to Advanced Communication Patterns

The Robot Operating System 2 (ROS 2) provides a sophisticated communication infrastructure that goes far beyond the fundamental publish-subscribe, request-reply, and goal-oriented patterns. Advanced communication patterns include complex system architectures, cross-domain communication bridging, and integration with external systems. These patterns enable the creation of highly distributed, resilient, and scalable robotic systems that can operate across various domains and platforms.

In this comprehensive guide, we'll explore the intricate details of these advanced patterns, including bidirectional bridges between different communication domains, complex message routing mechanisms, and architectural patterns that enable seamless integration of heterogeneous systems. Understanding these advanced patterns is crucial for developing sophisticated robotic applications that must integrate with legacy systems, cloud services, or other robotic platforms.

The evolution from ROS 1 to ROS 2 introduced significant improvements in communication reliability, real-time performance, and cross-platform compatibility. These improvements enable new architectural possibilities, including distributed systems spanning multiple machines, real-time performance guarantees, and secure communication paradigms.

## Deep Technical Analysis of Communication Bridging

### ROS 2 to ROS 1 Bridge

One of the most critical advanced patterns is the bridge between ROS 2 and ROS 1. This bridging capability enables gradual migration from ROS 1 to ROS 2 while maintaining interoperability between existing ROS 1 nodes and new ROS 2 components. The bridge works by forwarding messages between the two systems, maintaining the illusion of a unified system despite the underlying architectural differences.

The ROS 1-ROS 2 bridge operates through a pair of processes that run concurrently:
- **ROS 1 Bridge Node**: Communicates with the ROS 1 master and nodes using the ROS 1 communication protocols
- **ROS 2 Bridge Node**: Communicates with the ROS 2 system using DDS-based communication

The bridge maintains topic mapping between the two systems, ensuring that messages published on a ROS 1 topic are transparently forwarded to the corresponding ROS 2 topic and vice versa.

```cpp
// Implementation of a custom bridge example
#include "rclcpp/rclcpp.hpp"
#include "rosbridge_suite/rosbridge_tcp.h"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <websocketpp/config/asio_no_tls_client.hpp>
#include <websocketpp/client.hpp>

class AdvancedBridgeNode : public rclcpp::Node
{
public:
    AdvancedBridgeNode() : Node("advanced_bridge_node")
    {
        // Initialize ROS 2 publishers and subscribers
        ros2_cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        ros2_sensor_sub_ = this->create_subscription<std_msgs::msg::String>(
            "sensor_data", 10,
            std::bind(&AdvancedBridgeNode::ros2_sensor_callback, this, std::placeholders::_1)
        );
        
        // Initialize ROS 1 bridge interface
        initialize_ros1_bridge();
        
        // Initialize websocket bridge for external communication
        initialize_websocket_bridge();
        
        // Initialize parameter bridge for cross-domain configuration
        initialize_parameter_bridge();
        
        RCLCPP_INFO(this->get_logger(), "Advanced bridge node initialized");
    }

private:
    void initialize_ros1_bridge()
    {
        // Setup ROS 1 bridge parameters
        this->declare_parameter("bridge_enable_ros1", true);
        this->declare_parameter("bridge_topics_mapping", "");
        this->declare_parameter("bridge_services_mapping", "");
        
        if (this->get_parameter("bridge_enable_ros1").as_bool()) {
            // Initialize ROS 1 bridge connection
            // In a real implementation, this would establish connection to ROS 1 master
            RCLCPP_INFO(this->get_logger(), "ROS 1 bridge enabled");
            
            // Example of topic mapping configuration
            auto topics_mapping = this->get_parameter("bridge_topics_mapping").as_string();
            parse_topic_mapping(topics_mapping);
        }
    }
    
    void initialize_websocket_bridge()
    {
        // Websocket bridge for external communication (web browsers, external systems)
        this->declare_parameter("websocket_port", 9090);
        this->declare_parameter("websocket_address", "0.0.0.0");
        
        websocket_port_ = this->get_parameter("websocket_port").as_int();
        websocket_address_ = this->get_parameter("websocket_address").as_string();
        
        // Setup websocket server (simplified)
        setup_websocket_server();
    }
    
    void initialize_parameter_bridge()
    {
        // Bridge parameters between different domains
        this->declare_parameter("enable_parameter_bridge", true);
        this->declare_parameter("parameter_mapping", "");
        
        if (this->get_parameter("enable_parameter_bridge").as_bool()) {
            parameter_bridge_enabled_ = true;
            RCLCPP_INFO(this->get_logger(), "Parameter bridge enabled");
        }
    }
    
    void ros2_sensor_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        // Forward ROS 2 message to ROS 1 and external systems
        RCLCPP_INFO(this->get_logger(), "Received ROS 2 sensor data: %s", msg->data.c_str());
        
        // Forward to ROS 1 (if bridge is active)
        if (ros1_connected_) {
            forward_to_ros1("sensor_data", msg);
        }
        
        // Forward to websocket (for web visualization)
        if (websocket_connected_) {
            forward_via_websocket("sensor_data", msg);
        }
        
        // Store in parameter bridge if configured
        if (parameter_bridge_enabled_) {
            update_parameter_bridge("sensor_data_last", msg->data);
        }
    }
    
    void forward_to_ros1(const std::string& topic, const std_msgs::msg::String::SharedPtr msg)
    {
        // In real implementation, this would forward message to ROS 1 system
        RCLCPP_DEBUG(this->get_logger(), "Forwarding to ROS 1: %s -> %s", topic.c_str(), msg->data.c_str());
    }
    
    void forward_via_websocket(const std::string& topic, const std_msgs::msg::String::SharedPtr msg)
    {
        // Forward message via websocket to external systems
        RCLCPP_DEBUG(this->get_logger(), "Forwarding via websocket: %s -> %s", topic.c_str(), msg->data.c_str());
    }
    
    void update_parameter_bridge(const std::string& param_name, const std::string& param_value)
    {
        // Update cross-domain parameter
        RCLCPP_DEBUG(this->get_logger(), "Updating parameter bridge: %s = %s", param_name.c_str(), param_value.c_str());
    }
    
    void parse_topic_mapping(const std::string& mapping_string)
    {
        // Parse topic mapping configuration
        RCLCPP_INFO(this->get_logger(), "Parsing topic mapping: %s", mapping_string.c_str());
    }
    
    void setup_websocket_server()
    {
        // Setup websocket server for external communication
        RCLCPP_INFO(this->get_logger(), "Initializing websocket server at %s:%d", 
                   websocket_address_.c_str(), websocket_port_);
    }
    
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr ros2_cmd_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ros2_sensor_sub_;
    
    int websocket_port_;
    std::string websocket_address_;
    bool ros1_connected_ = false;
    bool websocket_connected_ = false;
    bool parameter_bridge_enabled_ = false;
};
```

### Web Communication Bridge

Web communication bridges enable ROS 2 systems to interact with web-based applications and dashboards. This pattern is essential for remote monitoring, web-based teleoperation, and user interfaces that leverage modern web technologies.

The web bridge typically uses WebSockets for real-time bidirectional communication, JSON for message serialization, and a standardized message format that can represent ROS 2 messages in a web-friendly format.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64, Int32
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan, Image
from rosbridge_library.capabilities import subscribe, publish, advertise, service_call
from flask import Flask, render_template
from flask_socketio import SocketIO, emit
import json
import threading
import base64
import numpy as np
import time
from typing import Dict, Any, Optional
import asyncio

class WebBridgeNode(Node):
    def __init__(self):
        super().__init__('web_bridge_node')
        
        # Web server setup
        self.web_host = '0.0.0.0'
        self.web_port = self.declare_parameter('web_port', 5000).value
        self.websocket_port = self.declare_parameter('websocket_port', 9090).value
        
        # Initialize Flask app
        self.app = Flask(__name__)
        self.socketio = SocketIO(self.app, cors_allowed_origins="*", async_mode='gevent')
        
        # ROS 2 publishers and subscribers
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.status_pub = self.create_publisher(String, 'web_status', 10)
        
        # Internal message queues for bridging
        self.message_queues = {}
        
        # Client connections tracking
        self.clients = {}
        
        # Setup message handlers
        self.setup_websocket_handlers()
        
        # Start web server in background thread
        self.start_web_server()
        
        # Start message processing timer
        self.message_timer = self.create_timer(0.1, self.process_web_messages)
        
        self.get_logger().info(f'Web bridge initialized on port {self.web_port}')
    
    def setup_websocket_handlers(self):
        """Setup WebSocket event handlers"""
        
        @self.socketio.on('connect')
        def handle_connect():
            client_id = request.sid
            self.clients[client_id] = {
                'connected_at': time.time(),
                'subscriptions': [],
                'last_heartbeat': time.time()
            }
            self.get_logger().info(f'Web client connected: {client_id}')
            
            # Send welcome message
            emit('connection_status', {
                'status': 'connected',
                'client_id': client_id,
                'timestamp': time.time()
            })
        
        @self.socketio.on('disconnect')
        def handle_disconnect():
            client_id = request.sid
            if client_id in self.clients:
                del self.clients[client_id]
            self.get_logger().info(f'Web client disconnected: {client_id}')
        
        @self.socketio.on('subscribe')
        def handle_subscribe(data):
            client_id = request.sid
            topic = data.get('topic', '')
            queue_name = data.get('queue_name', f'web_queue_{len(self.message_queues)}')
            
            if client_id in self.clients:
                self.clients[client_id]['subscriptions'].append(topic)
            
            # Create message queue for this subscription
            self.message_queues[queue_name] = {
                'topic': topic,
                'messages': [],
                'client_id': client_id
            }
            
            emit('subscription_ack', {
                'topic': topic,
                'queue': queue_name,
                'status': 'subscribed'
            })
        
        @self.socketio.on('unsubscribe')
        def handle_unsubscribe(data):
            topic = data.get('topic', '')
            client_id = request.sid
            
            if client_id in self.clients:
                if topic in self.clients[client_id]['subscriptions']:
                    self.clients[client_id]['subscriptions'].remove(topic)
            
            emit('unsubscribe_ack', {'topic': topic, 'status': 'unsubscribed'})
        
        @self.socketio.on('publish')
        def handle_publish(data):
            """Handle messages published from web client"""
            topic = data.get('topic', '')
            message_data = data.get('message', {})
            
            # Convert web message to ROS message
            ros_msg = self.convert_web_to_ros(topic, message_data)
            
            if ros_msg is not None:
                self.publish_to_ros(topic, ros_msg)
                self.get_logger().info(f'Published from web: {topic}')
        
        @self.socketio.on('teleop_command')
        def handle_teleop_command(data):
            """Handle teleoperation commands from web interface"""
            linear_speed = data.get('linear', 0.0)
            angular_speed = data.get('angular', 0.0)
            
            # Create Twist message for robot movement
            cmd_msg = Twist()
            cmd_msg.linear.x = float(linear_speed)
            cmd_msg.angular.z = float(angular_speed)
            
            self.cmd_pub.publish(cmd_msg)
            
            # Echo command back to client
            emit('command_echo', {
                'linear': linear_speed,
                'angular': angular_speed,
                'timestamp': time.time()
            })
        
        @self.socketio.on('parameter_update')
        def handle_parameter_update(data):
            """Handle parameter updates from web interface"""
            param_name = data.get('name', '')
            param_value = data.get('value', None)
            
            # In a real implementation, this would update ROS parameters
            self.get_logger().info(f'Parameter update: {param_name} = {param_value}')
            
            emit('parameter_ack', {
                'name': param_name,
                'value': param_value,
                'status': 'updated'
            })
    
    def convert_web_to_ros(self, topic: str, web_data: Dict[str, Any]) -> Optional[Any]:
        """Convert web message format to ROS message"""
        try:
            if topic == '/cmd_vel':
                cmd_msg = Twist()
                cmd_msg.linear.x = float(web_data.get('linear_x', 0.0))
                cmd_msg.linear.y = float(web_data.get('linear_y', 0.0))
                cmd_msg.linear.z = float(web_data.get('linear_z', 0.0))
                cmd_msg.angular.x = float(web_data.get('angular_x', 0.0))
                cmd_msg.angular.y = float(web_data.get('angular_y', 0.0))
                cmd_msg.angular.z = float(web_data.get('angular_z', 0.0))
                return cmd_msg
            
            elif topic == '/web_status':
                status_msg = String()
                status_msg.data = str(web_data.get('message', ''))
                return status_msg
            
            # Add more topic conversions as needed
            self.get_logger().warn(f'Unsupported topic conversion: {topic}')
            return None
            
        except Exception as e:
            self.get_logger().error(f'Error converting web message: {str(e)}')
            return None
    
    def publish_to_ros(self, topic: str, message):
        """Publish message to ROS 2 topic"""
        try:
            if topic == '/cmd_vel':
                self.cmd_pub.publish(message)
            elif topic == '/web_status':
                self.status_pub.publish(message)
            # Add more topic publishers as needed
        except Exception as e:
            self.get_logger().error(f'Error publishing to ROS: {str(e)}')
    
    def process_web_messages(self):
        """Process messages from ROS 2 and forward to web clients"""
        # This method would normally receive messages from ROS 2 topics
        # For demonstration, we'll simulate periodic status updates
        status_msg = String()
        status_msg.data = f"System OK - Connected clients: {len(self.clients)}"
        self.status_pub.publish(status_msg)
    
    def start_web_server(self):
        """Start web server in background thread"""
        def run_web_server():
            self.socketio.run(
                self.app,
                host=self.web_host,
                port=self.web_port,
                debug=False,
                use_reloader=False
            )
        
        web_thread = threading.Thread(target=run_web_server)
        web_thread.daemon = True
        web_thread.start()

class RosToWebBridge:
    """Bridge from ROS 2 topics to web clients"""
    
    def __init__(self, node: WebBridgeNode):
        self.node = node
        self.topic_subscribers = {}
        
        # Topics to bridge to web
        self.topics_to_bridge = [
            ('/web_status', String),
            ('/scan', LaserScan),
            ('/camera/image_raw', Image),
            ('/odom', Point)
        ]
    
    def start_bridge(self):
        """Start the topic bridging"""
        for topic_name, msg_type in self.topics_to_bridge:
            self.node.get_logger().info(f'Setting up bridge for {topic_name}')
            
            # Create subscriber for each topic
            sub = self.node.create_subscription(
                msg_type,
                topic_name,
                lambda msg, t=topic_name: self.ros_to_web_converter(msg, t),
                10
            )
            self.topic_subscribers[topic_name] = sub
    
    def ros_to_web_converter(self, msg, topic_name: str):
        """Convert ROS message to web format and send to clients"""
        
        # Convert ROS message to web-friendly format
        web_format = self.convert_ros_to_web(msg, topic_name)
        
        if web_format is not None:
            # Emit to all connected web clients
            for client_id in self.node.clients:
                self.node.socketio.emit('ros_message', {
                    'topic': topic_name,
                    'message': web_format,
                    'timestamp': time.time()
                }, room=client_id)
    
    def convert_ros_to_web(self, msg, topic_name: str) -> Optional[Dict[str, Any]]:
        """Convert ROS message to web format"""
        try:
            if topic_name == '/web_status' and hasattr(msg, 'data'):
                return {
                    'type': 'status',
                    'data': str(msg.data),
                    'timestamp': time.time()
                }
            
            elif topic_name == '/scan' and hasattr(msg, 'ranges'):
                return {
                    'type': 'laser_scan',
                    'ranges': [float(r) if r < float('inf') else 100.0 for r in msg.ranges[:180]],  # First 180 ranges
                    'angle_min': float(msg.angle_min),
                    'angle_max': float(msg.angle_max),
                    'angle_increment': float(msg.angle_increment)
                }
            
            elif topic_name == '/camera/image_raw' and hasattr(msg, 'data'):
                # Convert image to base64 for web
                image_data = base64.b64encode(msg.data).decode('utf-8')
                return {
                    'type': 'image',
                    'encoding': str(msg.encoding),
                    'height': int(msg.height),
                    'width': int(msg.width),
                    'data': image_data
                }
            
            elif topic_name == '/odom' and hasattr(msg, 'x'):
                return {
                    'type': 'position',
                    'x': float(msg.x),
                    'y': float(msg.y),
                    'z': float(msg.z)
                }
            
            # Add more conversions as needed
            self.node.get_logger().warn(f'Unsupported message conversion for: {topic_name}')
            return None
            
        except Exception as e:
            self.node.get_logger().error(f'Error converting ROS message to web: {str(e)}')
            return None

def main():
    rclpy.init()
    
    # Create bridge node
    bridge_node = WebBridgeNode()
    ros_to_web_bridge = RosToWebBridge(bridge_node)
    
    # Start ROS to web bridging
    ros_to_web_bridge.start_bridge()
    
    try:
        rclpy.spin(bridge_node)
    except KeyboardInterrupt:
        bridge_node.get_logger().info('Shutting down web bridge')
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Multi-Agent System Bridging

Multi-agent systems require specialized bridging patterns to coordinate multiple autonomous agents, each potentially running on different platforms or communication frameworks. This pattern is crucial for swarm robotics, distributed sensor networks, and multi-robot systems.

```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "multirobot_coordinator_msgs/msg/coordination_command.hpp"
#include "multirobot_coordinator_msgs/msg/agent_status.hpp"
#include <map>
#include <vector>
#include <mutex>
#include <algorithm>

class MultiAgentBridgeNode : public rclcpp::Node
{
public:
    struct AgentInfo {
        std::string id;
        std::string type;
        bool active;
        rclcpp::Time last_seen;
        std::string status;
        std::vector<std::string> capabilities;
        std::string ip_address;
        int port;
    };

    explicit MultiAgentBridgeNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("multi_agent_bridge", options)
    {
        // Parameters for multi-agent coordination
        this->declare_parameter("agent_discovery_interval", 5.0);
        this->declare_parameter("agent_timeout", 30.0);
        this->declare_parameter("coordinator_address", "localhost");
        this->declare_parameter("coordinator_port", 8080);
        
        discovery_interval_ = this->get_parameter("agent_discovery_interval").as_double();
        agent_timeout_ = this->get_parameter("agent_timeout").as_double();
        
        // Publishers and subscribers for agent coordination
        agent_status_pub_ = this->create_publisher<multirobot_coordinator_msgs::msg::AgentStatus>(
            "agent_status", 100
        );
        coordination_cmd_pub_ = this->create_publisher<multirobot_coordinator_msgs::msg::CoordinationCommand>(
            "coordination_commands", 100
        );
        
        agent_status_sub_ = this->create_subscription<multirobot_coordinator_msgs::msg::AgentStatus>(
            "agent_status", 100,
            std::bind(&MultiAgentBridgeNode::agent_status_callback, this, std::placeholders::_1)
        );
        
        // Setup discovery timer
        discovery_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(discovery_interval_),
            std::bind(&MultiAgentBridgeNode::discovery_timer_callback, this)
        );
        
        // Setup cleanup timer
        cleanup_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(5.0),
            std::bind(&MultiAgentBridgeNode::cleanup_old_agents, this)
        );
        
        RCLCPP_INFO(this->get_logger(), "Multi-agent bridge initialized");
    }

private:
    void agent_status_callback(const multirobot_coordinator_msgs::msg::AgentStatus::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(agents_mutex_);
        
        // Update or add agent info
        AgentInfo agent_info;
        agent_info.id = msg->agent_id;
        agent_info.type = msg->agent_type;
        agent_info.active = msg->active;
        agent_info.last_seen = this->get_clock()->now();
        agent_info.status = msg->status;
        agent_info.capabilities = msg->capabilities;
        
        agent_registry_[msg->agent_id] = agent_info;
        
        RCLCPP_DEBUG(this->get_logger(), 
                    "Agent status updated: %s - Active: %s", 
                    msg->agent_id.c_str(), 
                    msg->active ? "YES" : "NO");
    }
    
    void discovery_timer_callback()
    {
        // Broadcast discovery request to find new agents
        auto discovery_msg = multirobot_coordinator_msgs::msg::CoordinationCommand();
        discovery_msg.command = "DISCOVERY_REQUEST";
        discovery_msg.timestamp = this->get_clock()->now();
        
        coordination_cmd_pub_->publish(discovery_msg);
        
        // Also perform active agent discovery (implementation dependent)
        discover_new_agents();
    }
    
    void discover_new_agents()
    {
        std::lock_guard<std::mutex> lock(agents_mutex_);
        
        // In a real implementation, this would actively probe network addresses
        // or listen for beacon packets from agents
        RCLCPP_DEBUG(this->get_logger(), "Performing agent discovery...");
        
        // Example: Scan for agents on known ports/network segments
        for (const auto& [id, agent] : agent_registry_) {
            if (should_refresh_agent(agent)) {
                request_agent_status(id);
            }
        }
    }
    
    bool should_refresh_agent(const AgentInfo& agent) const
    {
        auto now = this->get_clock()->now();
        auto time_since_seen = (now - agent.last_seen).seconds();
        return time_since_seen > 10.0; // Refresh if not seen in 10 seconds
    }
    
    void request_agent_status(const std::string& agent_id)
    {
        auto status_request = multirobot_coordinator_msgs::msg::CoordinationCommand();
        status_request.command = "REQUEST_STATUS";
        status_request.target_agent_id = agent_id;
        status_request.timestamp = this->get_clock()->now();
        
        coordination_cmd_pub_->publish(status_request);
    }
    
    void cleanup_old_agents()
    {
        std::lock_guard<std::mutex> lock(agents_mutex_);
        
        auto now = this->get_clock()->now();
        auto it = agent_registry_.begin();
        
        while (it != agent_registry_.end()) {
            auto time_since_seen = (now - it->second.last_seen).seconds();
            
            if (time_since_seen > agent_timeout_) {
                RCLCPP_WARN(this->get_logger(), 
                           "Removing inactive agent: %s (inactive for %.1f seconds)", 
                           it->first.c_str(), time_since_seen);
                it = agent_registry_.erase(it);
            } else {
                ++it;
            }
        }
    }
    
    void assign_tasks_to_agents()
    {
        std::lock_guard<std::mutex> lock(agents_mutex_);
        
        // Simple round-robin task assignment algorithm
        std::vector<std::string> active_agents;
        for (const auto& [id, agent] : agent_registry_) {
            if (agent.active && agent.status == "READY") {
                active_agents.push_back(id);
            }
        }
        
        if (!active_agents.empty()) {
            // Assign tasks based on agent capabilities and current workload
            for (const auto& agent_id : active_agents) {
                assign_task_to_agent(agent_id);
            }
        }
    }
    
    void assign_task_to_agent(const std::string& agent_id)
    {
        // In a real implementation, this would assign specific tasks
        // based on agent capabilities, current state, and task requirements
        auto task_assignment = multirobot_coordinator_msgs::msg::CoordinationCommand();
        task_assignment.command = "TASK_ASSIGNMENT";
        task_assignment.target_agent_id = agent_id;
        task_assignment.parameters["task_type"] = "EXPLORATION";
        task_assignment.parameters["priority"] = "HIGH";
        task_assignment.timestamp = this->get_clock()->now();
        
        coordination_cmd_pub_->publish(task_assignment);
    }
    
    // Publishers and subscribers
    rclcpp::Publisher<multirobot_coordinator_msgs::msg::AgentStatus>::SharedPtr agent_status_pub_;
    rclcpp::Publisher<multirobot_coordinator_msgs::msg::CoordinationCommand>::SharedPtr coordination_cmd_pub_;
    rclcpp::Subscription<multirobot_coordinator_msgs::msg::AgentStatus>::SharedPtr agent_status_sub_;
    
    // Timers
    rclcpp::TimerBase::SharedPtr discovery_timer_;
    rclcpp::TimerBase::SharedPtr cleanup_timer_;
    
    // Agent registry and management
    std::map<std::string, AgentInfo> agent_registry_;
    std::mutex agents_mutex_;
    
    // Configuration parameters
    double discovery_interval_;
    double agent_timeout_;
};
```

## Cross-Platform Communication Bridging

### DDS Bridge Implementation

Data Distribution Service (DDS) bridges enable communication between different DDS implementations or between DDS and other middleware systems. This is crucial for integrating ROS 2 with industrial systems, aerospace applications, or other DDS-based systems.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Float64
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import LaserScan
import threading
import time
import open_splice_ddsi  # Hypothetical DDS interface
import cyclone_drs  # Another hypothetical DDS interface
from typing import Dict, Any, Optional, List
import json

class DDSBridgeNode(Node):
    def __init__(self):
        super().__init__('dds_bridge_node')
        
        # DDS configurations
        self.dds_configs = self.declare_parameter('dds_configs', []).value
        
        # ROS publishers/subscribers
        self.ros_publishers = {}
        self.ros_subscribers = {}
        
        # DDS participants and entities
        self.dds_participants = {}
        self.dds_readers = {}
        self.dds_writers = {}
        
        # Message transformation rules
        self.transformation_rules = {}
        
        # Bridging configuration
        self.bridge_config = {
            'enable_qos_mapping': True,
            'enable_type_conversion': True,
            'enable_security': False,
            'sync_buffers': True
        }
        
        self.initialize_bridging()
        
        # Start bridging process
        self.start_bridging_process()
        
        self.get_logger().info('DDS bridge node initialized')
    
    def initialize_bridging(self):
        """Initialize the DDS bridging configuration"""
        # Initialize ROS side
        self.setup_ros_interfaces()
        
        # Initialize DDS side
        self.setup_dds_interfaces()
        
        # Setup transformation rules
        self.setup_transformation_rules()
    
    def setup_ros_interfaces(self):
        """Setup ROS 2 publishers and subscribers"""
        # Common topics to bridge
        ros_topics = [
            ('/cmd_vel', Twist),
            ('/odom', Pose),
            ('/scan', LaserScan),
            ('/status', String)
        ]
        
        for topic_name, msg_type in ros_topics:
            # Create publisher for bridging back from DDS
            pub = self.create_publisher(msg_type, f'{topic_name}_from_dds', 10)
            self.ros_publishers[topic_name] = pub
            
            # Create subscriber for bridging to DDS
            sub = self.create_subscription(
                msg_type,
                topic_name,
                lambda msg, t=topic_name: self.ros_to_dds_forwarder(msg, t),
                10
            )
            self.ros_subscribers[topic_name] = sub
    
    def setup_dds_interfaces(self):
        """Setup DDS participants, readers, and writers"""
        for config in self.dds_configs:
            domain_id = config.get('domain_id', 0)
            participant_name = config.get('participant_name', 'default_participant')
            
            # Create DDS participant
            participant = self.create_dds_participant(domain_id, participant_name)
            self.dds_participants[participant_name] = participant
            
            # Setup DDS readers and writers based on configuration
            for topic_config in config.get('topics', []):
                topic_name = topic_config['name']
                dds_type = topic_config['type']
                
                # Create DDS writer
                writer = self.create_dds_writer(participant, topic_name, dds_type)
                self.dds_writers[f'{participant_name}.{topic_name}'] = writer
                
                # Create DDS reader
                reader = self.create_dds_reader(participant, topic_name, dds_type)
                self.dds_readers[f'{participant_name}.{topic_name}'] = reader
    
    def create_dds_participant(self, domain_id: int, name: str):
        """Create DDS participant"""
        # In real implementation, this would create actual DDS participant
        # For demo purposes, return a mock object
        return MockDDSParticipant(domain_id, name)
    
    def create_dds_writer(self, participant, topic_name: str, dds_type: str):
        """Create DDS data writer"""
        # In real implementation, this would create actual DDS writer
        return MockDDSWriter(participant, topic_name, dds_type)
    
    def create_dds_reader(self, participant, topic_name: str, dds_type: str):
        """Create DDS data reader"""
        # In real implementation, this would create actual DDS reader
        return MockDDSReader(participant, topic_name, dds_type, self.dds_message_handler)
    
    def setup_transformation_rules(self):
        """Setup rules for transforming between ROS and DDS messages"""
        # Example transformation rules
        self.transformation_rules = {
            'Twist_to_DDS_Command': {
                'mapping': {
                    'linear.x': 'cmd.linear.x',
                    'linear.y': 'cmd.linear.y',
                    'angular.z': 'cmd.angular.z'
                },
                'validator': self.validate_twist_transformation
            },
            'DDS_Scan_to_LaserScan': {
                'mapping': {
                    'ranges': 'scan.ranges',
                    'intensities': 'scan.intensities',
                    'angle_min': 'scan.angle_min'
                },
                'validator': self.validate_scan_transformation
            }
        }
    
    def ros_to_dds_forwarder(self, msg, topic_name: str):
        """Forward ROS message to DDS"""
        try:
            # Transform ROS message to DDS format
            dds_msg = self.transform_ros_to_dds(msg, topic_name)
            
            if dds_msg is not None:
                # Find appropriate DDS writer
                for key, writer in self.dds_writers.items():
                    if topic_name in key:
                        writer.write(dds_msg)
                        break
                
                self.get_logger().debug(f'Forwarded ROS message to DDS: {topic_name}')
        
        except Exception as e:
            self.get_logger().error(f'Error forwarding ROS to DDS: {str(e)}')
    
    def dds_message_handler(self, dds_msg, topic_name: str):
        """Handle incoming DDS message and forward to ROS"""
        try:
            # Transform DDS message to ROS format
            ros_msg = self.transform_dds_to_ros(dds_msg, topic_name)
            
            if ros_msg is not None:
                # Find appropriate ROS publisher
                pub_key = f'{topic_name}_from_dds'
                if pub_key in self.ros_publishers:
                    self.ros_publishers[pub_key].publish(ros_msg)
                
                self.get_logger().debug(f'Forwarded DDS message to ROS: {topic_name}')
        
        except Exception as e:
            self.get_logger().error(f'Error forwarding DDS to ROS: {str(e)}')
    
    def transform_ros_to_dds(self, ros_msg, topic_name: str):
        """Transform ROS message to DDS format"""
        # This would contain actual transformation logic
        # For demo, return a mock transformation
        if topic_name == '/cmd_vel':
            return self.create_mock_dds_command(ros_msg)
        elif topic_name == '/scan':
            return self.create_mock_dds_scan(ros_msg)
        
        return None
    
    def transform_dds_to_ros(self, dds_msg, topic_name: str):
        """Transform DDS message to ROS format"""
        # This would contain actual transformation logic
        if hasattr(dds_msg, 'cmd') and hasattr(dds_msg.cmd, 'linear'):
            from geometry_msgs.msg import Twist
            ros_msg = Twist()
            ros_msg.linear.x = getattr(dds_msg.cmd.linear, 'x', 0.0)
            ros_msg.angular.z = getattr(dds_msg.cmd.angular, 'z', 0.0)
            return ros_msg
        
        return None
    
    def create_mock_dds_command(self, ros_msg):
        """Create mock DDS command from ROS Twist message"""
        # This is a mock implementation
        class MockDDSCommand:
            def __init__(self, linear_x=0.0, angular_z=0.0):
                class Linear:
                    x = linear_x
                class Angular:
                    z = angular_z
                self.linear = Linear()
                self.angular = Angular()
        
        return MockDDSCommand(ros_msg.linear.x, ros_msg.angular.z)
    
    def validate_twist_transformation(self, ros_msg, dds_msg) -> bool:
        """Validate twist transformation"""
        try:
            if not hasattr(ros_msg, 'linear') or not hasattr(ros_msg, 'angular'):
                return False
            
            linear_x = ros_msg.linear.x if hasattr(ros_msg.linear, 'x') else 0.0
            angular_z = ros_msg.angular.z if hasattr(ros_msg.angular, 'z') else 0.0
            
            # Validate range
            if abs(linear_x) > 5.0 or abs(angular_z) > 3.14:
                self.get_logger().warn('Command values out of expected range')
            
            return True
        except:
            return False
    
    def start_bridging_process(self):
        """Start the bridging process in background thread"""
        def bridge_worker():
            while rclpy.ok():
                try:
                    # Process DDS data (this would involve DDS-specific polling)
                    self.process_dds_data()
                    
                    # Process any buffered data
                    self.process_buffered_messages()
                    
                    time.sleep(0.01)  # 100 Hz
                except Exception as e:
                    self.get_logger().error(f'Bridging worker error: {str(e)}')
        
        self.bridge_thread = threading.Thread(target=bridge_worker)
        self.bridge_thread.daemon = True
        self.bridge_thread.start()
    
    def process_dds_data(self):
        """Process incoming DDS data"""
        # In real implementation, this would poll DDS readers
        pass
    
    def process_buffered_messages(self):
        """Process any buffered messages"""
        # Process queued messages if any
        pass

class MockDDSParticipant:
    def __init__(self, domain_id, name):
        self.domain_id = domain_id
        self.name = name
        self.entities = []

class MockDDSWriter:
    def __init__(self, participant, topic_name, dds_type):
        self.participant = participant
        self.topic_name = topic_name
        self.dds_type = dds_type
    
    def write(self, message):
        # Mock write implementation
        pass

class MockDDSReader:
    def __init__(self, participant, topic_name, dds_type, callback):
        self.participant = participant
        self.topic_name = topic_name
        self.dds_type = dds_type
        self.callback = callback

def main():
    rclpy.init()
    
    dds_bridge = DDSBridgeNode()
    
    try:
        rclpy.spin(dds_bridge)
    except KeyboardInterrupt:
        dds_bridge.get_logger().info('Shutting down DDS bridge')
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Advanced Message Routing and Filtering

### Content-Based Message Routing

Content-based routing allows messages to be forwarded based on their content, enabling sophisticated filtering and routing mechanisms:

```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <regex>
#include <functional>
#include <map>

class ContentBasedRouter : public rclcpp::Node
{
public:
    struct RouteRule {
        std::string pattern;  // Regex pattern for matching
        std::string destination_topic;
        std::function<bool(const std::string&)> condition;  // Additional condition
        bool priority_route;  // Whether this is a priority route
    };

    explicit ContentBasedRouter(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("content_based_router", options)
    {
        // Initialize route rules
        initialize_routing_rules();
        
        // Setup subscribers
        message_sub_ = this->create_subscription<std_msgs::msg::String>(
            "input_stream", 100,
            std::bind(&ContentBasedRouter::message_callback, this, std::placeholders::_1)
        );
        
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan_input", 100,
            std::bind(&ContentBasedRouter::laser_callback, this, std::placeholders::_1)
        );
        
        // Setup publishers for routed messages
        setup_routed_publishers();
        
        RCLCPP_INFO(this->get_logger(), "Content-based router initialized with %zu rules", routing_rules_.size());
    }

private:
    void initialize_routing_rules()
    {
        // Rule 1: Error messages to error channel
        RouteRule error_rule;
        error_rule.pattern = ".*ERROR.*|.*FATAL.*|.*CRITICAL.*";
        error_rule.destination_topic = "error_log";
        error_rule.condition = nullptr;
        error_rule.priority_route = true;
        routing_rules_.push_back(error_rule);
        
        // Rule 2: Warning messages to warning channel
        RouteRule warning_rule;
        warning_rule.pattern = ".*WARNING.*|.*WARN.*";
        warning_rule.destination_topic = "warning_log";
        warning_rule.condition = nullptr;
        warning_rule.priority_route = false;
        routing_rules_.push_back(warning_rule);
        
        // Rule 3: High-priority messages
        RouteRule high_priority_rule;
        high_priority_rule.pattern = ".*HIGH_PRIORITY.*";
        high_priority_rule.destination_topic = "priority_messages";
        high_priority_rule.condition = nullptr;
        high_priority_rule.priority_route = true;
        routing_rules_.push_back(high_priority_rule);
        
        // Rule 4: Sensor data with specific conditions
        RouteRule sensor_rule;
        sensor_rule.pattern = ".*SENSOR.*";
        sensor_rule.destination_topic = "sensor_processed";
        sensor_rule.condition = [this](const std::string& msg) -> bool {
            return msg.length() > 10;  // Only route if message is long enough
        };
        sensor_rule.priority_route = false;
        routing_rules_.push_back(sensor_rule);
        
        // Rule 5: Debug messages (lower priority)
        RouteRule debug_rule;
        debug_rule.pattern = ".*DEBUG.*|.*VERBOSE.*";
        debug_rule.destination_topic = "debug_log";
        debug_rule.condition = nullptr;
        debug_rule.priority_route = false;
        routing_rules_.push_back(debug_rule);
    }
    
    void setup_routed_publishers()
    {
        // Create publishers for each destination topic
        for (const auto& rule : routing_rules_) {
            if (publisher_map_.find(rule.destination_topic) == publisher_map_.end()) {
                auto pub = this->create_publisher<std_msgs::msg::String>(rule.destination_topic, 100);
                publisher_map_[rule.destination_topic] = pub;
            }
        }
    }
    
    void message_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::string content = msg->data;
        bool routed = false;
        
        // Apply rules in priority order
        for (const auto& rule : routing_rules_) {
            if (std::regex_search(content, std::regex(rule.pattern))) {
                // Check additional condition if provided
                if (rule.condition && !rule.condition(content)) {
                    continue;  // Skip if condition not met
                }
                
                // Route the message
                auto routed_msg = std::make_shared<std_msgs::msg::String>();
                routed_msg->data = format_routed_message(content, rule.destination_topic);
                
                auto it = publisher_map_.find(rule.destination_topic);
                if (it != publisher_map_.end()) {
                    it->second->publish(routed_msg);
                    RCLCPP_DEBUG(this->get_logger(), "Routed message to %s: %s", 
                                rule.destination_topic.c_str(), content.c_str());
                    routed = true;
                    
                    // If it's a priority route, don't continue to other rules
                    if (rule.priority_route) {
                        break;
                    }
                }
            }
        }
        
        if (!routed) {
            // Default routing for unmatched messages
            auto default_msg = std::make_shared<std_msgs::msg::String>();
            default_msg->data = "[DEFAULT] " + content;
            default_publisher_->publish(default_msg);
        }
    }
    
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
    {
        // Process laser scan data and route based on content/analysis
        std::stringstream scan_summary;
        scan_summary << "LASER_SCAN "
                     << "Ranges: " << scan_msg->ranges.size()
                     << " MinRange: " << scan_msg->range_min
                     << " MaxRange: " << scan_msg->range_max;
        
        std::string scan_content = scan_summary.str();
        
        // Apply routing rules to laser data
        for (const auto& rule : routing_rules_) {
            if (std::regex_search(scan_content, std::regex(rule.pattern))) {
                if (rule.condition && !rule.condition(scan_content)) {
                    continue;
                }
                
                auto routed_msg = std::make_shared<std_msgs::msg::String>();
                routed_msg->data = scan_content;
                
                auto it = publisher_map_.find(rule.destination_topic);
                if (it != publisher_map_.end()) {
                    it->second->publish(routed_msg);
                    RCLCPP_DEBUG(this->get_logger(), "Routed laser data to %s", rule.destination_topic.c_str());
                    break;
                }
            }
        }
    }
    
    std::string format_routed_message(const std::string& original, const std::string& destination)
    {
        return "[ROUTED:" + destination + "] " + original;
    }
    
    std::vector<RouteRule> routing_rules_;
    std::map<std::string, rclcpp::Publisher<std_msgs::msg::String>::SharedPtr> publisher_map_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr default_publisher_;
    
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr message_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
};
```

## Bridging with External Systems

### Cloud Service Bridging

Bridging ROS 2 with cloud services requires careful consideration of network protocols, data serialization, and security:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Float64
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
import requests
import json
import threading
import time
import base64
import io
from datetime import datetime
from typing import Dict, Any, Optional
import asyncio
from aiohttp import ClientSession, ClientError

class CloudBridgeNode(Node):
    def __init__(self):
        super().__init__('cloud_bridge_node')
        
        # Cloud configuration
        self.cloud_config = {
            'api_base_url': self.declare_parameter('api_base_url', 'https://api.example.com').value,
            'auth_token': self.declare_parameter('auth_token', '').value,
            'cloud_topics': self.declare_parameter('cloud_topics', []).value,
            'upload_interval': self.declare_parameter('upload_interval', 30.0).value,
            'batch_size': self.declare_parameter('batch_size', 10).value,
            'connection_timeout': self.declare_parameter('connection_timeout', 10.0).value
        }
        
        # Authentication and security
        self.auth_headers = {
            'Authorization': f'Bearer {self.cloud_config["auth_token"]}',
            'Content-Type': 'application/json',
            'User-Agent': 'ROS2-Cloud-Bridge/1.0'
        }
        
        # Data buffers for batching
        self.data_buffers = {}
        self.buffer_locks = {}
        
        # Publishers and subscribers
        self.ros_subscribers = {}
        self.setup_ros_interfaces()
        
        # Cloud communication timer
        self.upload_timer = self.create_timer(
            self.cloud_config['upload_interval'],
            self.upload_batched_data
        )
        
        # Connection monitoring
        self.connection_monitor_timer = self.create_timer(5.0, self.monitor_connection)
        
        # Statistics
        self.stats = {
            'messages_sent': 0,
            'messages_received': 0,
            'upload_errors': 0,
            'last_upload': 0
        }
        
        self.get_logger().info('Cloud bridge node initialized')
    
    def setup_ros_interfaces(self):
        """Setup ROS interfaces for cloud bridging"""
        cloud_topics = self.cloud_config['cloud_topics']
        
        for topic_info in cloud_topics:
            topic_name = topic_info['name']
            topic_type = topic_info['type']
            cloud_endpoint = topic_info['endpoint']
            upload_strategy = topic_info.get('strategy', 'immediate')
            
            # Create appropriate subscriber based on message type
            if topic_type == 'std_msgs/String':
                sub = self.create_subscription(
                    String,
                    topic_name,
                    lambda msg, t=topic_name, e=cloud_endpoint, s=upload_strategy: 
                        self.ros_message_handler(msg, t, e, s),
                    10
                )
            elif topic_type == 'sensor_msgs/Image':
                sub = self.create_subscription(
                    Image,
                    topic_name,
                    lambda msg, t=topic_name, e=cloud_endpoint, s=upload_strategy: 
                        self.image_message_handler(msg, t, e, s),
                    10
                )
            elif topic_type == 'sensor_msgs/LaserScan':
                sub = self.create_subscription(
                    LaserScan,
                    topic_name,
                    lambda msg, t=topic_name, e=cloud_endpoint, s=upload_strategy: 
                        self.laser_message_handler(msg, t, e, s),
                    10
                )
            elif topic_type == 'geometry_msgs/Twist':
                sub = self.create_subscription(
                    Twist,
                    topic_name,
                    lambda msg, t=topic_name, e=cloud_endpoint, s=upload_strategy: 
                        self.twist_message_handler(msg, t, e, s),
                    10
                )
            else:
                self.get_logger().warn(f'Unsupported message type: {topic_type}')
                continue
            
            self.ros_subscribers[topic_name] = {
                'subscriber': sub,
                'endpoint': cloud_endpoint,
                'strategy': upload_strategy,
                'buffer': [] if upload_strategy == 'batch' else None,
                'lock': threading.Lock()
            }
            
            # Initialize data buffer if using batch strategy
            if upload_strategy == 'batch':
                self.data_buffers[topic_name] = []
                self.buffer_locks[topic_name] = threading.Lock()
    
    def ros_message_handler(self, msg, topic_name: str, endpoint: str, strategy: str):
        """Handle standard ROS messages"""
        payload = {
            'topic': topic_name,
            'data': msg.data,
            'timestamp': datetime.utcnow().isoformat(),
            'ros_timestamp': self.get_clock().now().to_msg().sec,
            'source_node': self.get_name()
        }
        
        self.handle_message_for_cloud(payload, topic_name, endpoint, strategy)
    
    def image_message_handler(self, msg, topic_name: str, endpoint: str, strategy: str):
        """Handle image messages with compression"""
        # Convert image to base64 for JSON transport
        image_data = base64.b64encode(bytes(msg.data)).decode('utf-8')
        
        payload = {
            'topic': topic_name,
            'image': {
                'data': image_data,
                'encoding': msg.encoding,
                'height': msg.height,
                'width': msg.width,
                'step': msg.step
            },
            'timestamp': datetime.utcnow().isoformat(),
            'source_node': self.get_name()
        }
        
        self.handle_message_for_cloud(payload, topic_name, endpoint, strategy)
    
    def laser_message_handler(self, msg, topic_name: str, endpoint: str, strategy: str):
        """Handle laser scan messages"""
        payload = {
            'topic': topic_name,
            'laser_scan': {
                'ranges': [float(r) if r < float('inf') else 999.0 for r in msg.ranges[:100]],  # Limit for performance
                'intensities': [float(i) for i in msg.intensities],
                'angle_min': float(msg.angle_min),
                'angle_max': float(msg.angle_max),
                'angle_increment': float(msg.angle_increment),
                'time_increment': float(msg.time_increment),
                'scan_time': float(msg.scan_time),
                'range_min': float(msg.range_min),
                'range_max': float(msg.range_max)
            },
            'timestamp': datetime.utcnow().isoformat(),
            'source_node': self.get_name()
        }
        
        self.handle_message_for_cloud(payload, topic_name, endpoint, strategy)
    
    def twist_message_handler(self, msg, topic_name: str, endpoint: str, strategy: str):
        """Handle twist messages"""
        payload = {
            'topic': topic_name,
            'twist': {
                'linear': {
                    'x': float(msg.linear.x),
                    'y': float(msg.linear.y),
                    'z': float(msg.linear.z)
                },
                'angular': {
                    'x': float(msg.angular.x),
                    'y': float(msg.angular.y),
                    'z': float(msg.angular.z)
                }
            },
            'timestamp': datetime.utcnow().isoformat(),
            'source_node': self.get_name()
        }
        
        self.handle_message_for_cloud(payload, topic_name, endpoint, strategy)
    
    def handle_message_for_cloud(self, payload: Dict[str, Any], topic_name: str, endpoint: str, strategy: str):
        """Handle message processing based on strategy"""
        if strategy == 'immediate':
            # Send immediately
            self.send_to_cloud(endpoint, payload)
            self.stats['messages_sent'] += 1
        
        elif strategy == 'batch':
            # Add to batch buffer
            with self.buffer_locks[topic_name]:
                self.data_buffers[topic_name].append(payload)
                
                # Check if buffer is full
                if len(self.data_buffers[topic_name]) >= self.cloud_config['batch_size']:
                    self.send_batch(topic_name, endpoint)
    
    async def async_send_to_cloud(self, url: str, data: Dict[str, Any]) -> bool:
        """Asynchronously send data to cloud"""
        try:
            async with ClientSession(headers=self.auth_headers) as session:
                async with session.post(url, json=data, timeout=self.cloud_config['connection_timeout']) as response:
                    if response.status == 200:
                        self.get_logger().debug(f'Successfully sent data to cloud: {url}')
                        return True
                    else:
                        error_text = await response.text()
                        self.get_logger().error(f'Cloud upload failed: {response.status} - {error_text}')
                        return False
        except ClientError as e:
            self.get_logger().error(f'Network error during cloud upload: {str(e)}')
            self.stats['upload_errors'] += 1
            return False
        except Exception as e:
            self.get_logger().error(f'Unexpected error during cloud upload: {str(e)}')
            self.stats['upload_errors'] += 1
            return False
    
    def send_to_cloud(self, endpoint: str, data: Dict[str, Any]) -> bool:
        """Synchronous send to cloud"""
        try:
            url = f"{self.cloud_config['api_base_url']}{endpoint}"
            
            response = requests.post(
                url,
                json=data,
                headers=self.auth_headers,
                timeout=self.cloud_config['connection_timeout']
            )
            
            if response.status_code == 200:
                self.get_logger().debug(f'Successfully sent data to cloud: {endpoint}')
                return True
            else:
                self.get_logger().error(f'Cloud upload failed: {response.status_code} - {response.text}')
                return False
                
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f'Network error during cloud upload: {str(e)}')
            self.stats['upload_errors'] += 1
            return False
        except Exception as e:
            self.get_logger().error(f'Unexpected error during cloud upload: {str(e)}')
            self.stats['upload_errors'] += 1
            return False
    
    def send_batch(self, topic_name: str, endpoint: str):
        """Send batched data to cloud"""
        with self.buffer_locks[topic_name]:
            batch_data = self.data_buffers[topic_name][:]
            self.data_buffers[topic_name] = []  # Clear buffer
        
        if batch_data:
            payload = {
                'batch_id': f'{topic_name}_{datetime.utcnow().strftime("%Y%m%d_%H%M%S")}',
                'count': len(batch_data),
                'data': batch_data,
                'timestamp': datetime.utcnow().isoformat()
            }
            
            success = self.send_to_cloud(endpoint, payload)
            if success:
                self.stats['messages_sent'] += len(batch_data)
    
    def upload_batched_data(self):
        """Periodically send all batched data"""
        for topic_name, topic_info in self.ros_subscribers.items():
            if topic_info['strategy'] == 'batch':
                with self.buffer_locks[topic_name]:
                    if self.data_buffers[topic_name]:
                        self.send_batch(topic_name, topic_info['endpoint'])
        
        self.stats['last_upload'] = time.time()
        
        # Log periodic statistics
        self.get_logger().info(
            f'Cloud bridge stats - Sent: {self.stats["messages_sent"]}, '
            f'Errors: {self.stats["upload_errors"]}'
        )
    
    def monitor_connection(self):
        """Monitor cloud connection status"""
        try:
            ping_url = f"{self.cloud_config['api_base_url']}/ping"
            response = requests.get(ping_url, headers=self.auth_headers, timeout=5.0)
            
            if response.status_code == 200:
                self.get_logger().debug('Cloud connection healthy')
            else:
                self.get_logger().warn(f'Cloud connection issue: {response.status_code}')
                
        except Exception as e:
            self.get_logger().error(f'Cloud connection check failed: {str(e)}')
    
    def get_bridge_status(self) -> Dict[str, Any]:
        """Get current bridge status"""
        return {
            'stats': self.stats,
            'config': {
                'api_base_url': self.cloud_config['api_base_url'],
                'upload_interval': self.cloud_config['upload_interval'],
                'batch_size': self.cloud_config['batch_size']
            },
            'subscriptions': list(self.ros_subscribers.keys()),
            'buffer_sizes': {topic: len(buffer) if buffer else 0 
                           for topic, buffer in self.data_buffers.items()},
            'timestamp': datetime.utcnow().isoformat()
        }

def main():
    rclpy.init()
    
    cloud_bridge = CloudBridgeNode()
    
    try:
        rclpy.spin(cloud_bridge)
    except KeyboardInterrupt:
        cloud_bridge.get_logger().info('Shutting down cloud bridge')
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Advanced Bridging Patterns

### Bidirectional Real-Time Bridge

Creating a true bidirectional bridge with real-time guarantees requires careful attention to timing, synchronization, and resource management:

```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "realtime_tools/realtime_publisher.h"
#include "realtime_tools/realtime_buffer.h"
#include <thread>
#include <mutex>
#include <condition_variable>
#include <deque>
#include <chrono>

class RealtimeBridgeNode : public rclcpp::Node
{
public:
    struct MessageQueue {
        std::deque<geometry_msgs::msg::Twist> messages;
        std::mutex queue_mutex;
        std::condition_variable queue_condition;
        
        void push_back(const geometry_msgs::msg::Twist& msg) {
            std::lock_guard<std::mutex> lock(queue_mutex);
            messages.push_back(msg);
            queue_condition.notify_one();
        }
        
        bool try_pop_front(geometry_msgs::msg::Twist& msg) {
            std::lock_guard<std::mutex> lock(queue_mutex);
            if (!messages.empty()) {
                msg = messages.front();
                messages.pop_front();
                return true;
            }
            return false;
        }
        
        bool wait_and_pop_front(geometry_msgs::msg::Twist& msg, 
                               std::chrono::milliseconds timeout) {
            std::unique_lock<std::mutex> lock(queue_mutex);
            if (queue_condition.wait_for(lock, timeout, 
                                        [&] { return !messages.empty(); })) {
                msg = messages.front();
                messages.pop_front();
                return true;
            }
            return false;
        }
    };

    explicit RealtimeBridgeNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("realtime_bridge", options)
    {
        // Real-time parameters
        this->declare_parameter("realtime_priority", 80);
        this->declare_parameter("max_latency_ms", 10);
        this->declare_parameter("buffer_size", 100);
        
        realtime_priority_ = this->get_parameter("realtime_priority").as_int();
        max_latency_ms_ = this->get_parameter("max_latency_ms").as_int();
        buffer_size_ = this->get_parameter("buffer_size").as_int();
        
        // Setup publishers and subscribers
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_output", 10);
        cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_input", 10,
            std::bind(&RealtimeBridgeNode::cmd_callback, this, std::placeholders::_1)
        );
        
        // Setup message queues for real-time processing
        bridge_queue_ = std::make_unique<MessageQueue>();
        
        // Start real-time processing thread
        start_realtime_thread();
        
        RCLCPP_INFO(this->get_logger(), "Real-time bridge initialized with priority %d", realtime_priority_);
    }

private:
    void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // Add message to queue for real-time processing
        geometry_msgs::msg::Twist processed_msg = preprocess_message(*msg);
        bridge_queue_->push_back(processed_msg);
    }
    
    geometry_msgs::msg::Twist preprocess_message(const geometry_msgs::msg::Twist& input)
    {
        // Preprocess message with real-time constraints
        geometry_msgs::msg::Twist output = input;
        
        // Apply any real-time preprocessing (filtering, scaling, etc.)
        output.linear.x *= 1.0;  // No scaling in this example
        output.angular.z *= 1.0;
        
        return output;
    }
    
    void start_realtime_thread()
    {
        realtime_thread_ = std::thread([this]() {
            // Set real-time priority
            struct sched_param param;
            param.sched_priority = realtime_priority_;
            if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
                RCLCPP_WARN(this->get_logger(), "Could not set real-time priority");
            }
            
            RCLCPP_INFO(this->get_logger(), "Real-time thread started");
            
            geometry_msgs::msg::Twist msg;
            auto timeout = std::chrono::milliseconds(max_latency_ms_);
            
            while (rclcpp::ok()) {
                try {
                    // Wait for message with timeout
                    if (bridge_queue_->wait_and_pop_front(msg, timeout)) {
                        // Process message in real-time
                        auto processed_msg = process_realtime_message(msg);
                        
                        // Publish with real-time publisher
                        if (cmd_pub_->trylock()) {
                            *cmd_pub_->get_msg() = processed_msg;
                            cmd_pub_->unlockAndPublish();
                        }
                        
                        // Track latency
                        auto now = std::chrono::high_resolution_clock::now();
                        auto latency = std::chrono::duration_cast<std::chrono::microseconds>(
                            now - message_start_time_).count();
                        
                        if (latency > max_latency_ms_ * 1000) {
                            RCLCPP_WARN_THROTTLE(
                                this->get_logger(),
                                *this->get_clock(),
                                1000,  // throttle period in ms
                                "Real-time deadline exceeded: %ld μs", latency
                            );
                        }
                    } else {
                        // Timeout occurred - could implement watchdog behavior
                        RCLCPP_DEBUG_THROTTLE(
                            this->get_logger(),
                            *this->get_clock(),
                            5000,  // throttle period in ms
                            "Real-time queue timeout"
                        );
                    }
                    
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "Real-time thread error: %s", e.what());
                }
            }
            
            RCLCPP_INFO(this->get_logger(), "Real-time thread exiting");
        });
        
        // Detach the thread - it will run independently
        realtime_thread_.detach();
    }
    
    geometry_msgs::msg::Twist process_realtime_message(const geometry_msgs::msg::Twist& input)
    {
        // Process message in real-time context
        // This should be kept as lightweight as possible
        geometry_msgs::msg::Twist output = input;
        
        // Apply any real-time processing
        // Note: Avoid locks, dynamic allocations, system calls, etc.
        output.linear.x = std::clamp(input.linear.x, -2.0, 2.0);
        output.angular.z = std::clamp(input.angular.z, -1.5, 1.5);
        
        return output;
    }
    
    // Publishers and subscribers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    
    // Real-time processing
    std::unique_ptr<MessageQueue> bridge_queue_;
    std::thread realtime_thread_;
    
    // Configuration
    int realtime_priority_;
    int max_latency_ms_;
    int buffer_size_;
    
    // Timing
    std::chrono::high_resolution_clock::time_point message_start_time_;
};
```

## Bridging with Legacy Systems

### Protocol Adapter Pattern

Creating bridges to legacy systems often requires implementing protocol adapters that can translate between different communication protocols:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Float64
import socket
import struct
import threading
import time
from typing import Dict, Any, Optional, Callable
import xml.etree.ElementTree as ET

class LegacyProtocolAdapter(Node):
    def __init__(self):
        super().__init__('legacy_protocol_adapter')
        
        # Legacy system configuration
        self.legacy_config = {
            'protocol': self.declare_parameter('protocol', 'tcp').value,
            'host': self.declare_parameter('legacy_host', 'localhost').value,
            'port': self.declare_parameter('legacy_port', 12345).value,
            'timeout': self.declare_parameter('timeout', 5.0).value,
            'message_format': self.declare_parameter('message_format', 'binary').value
        }
        
        # Protocol-specific configuration
        self.protocol_handlers = {
            'modbus': self.handle_modbus_message,
            'opc_ua': self.handle_opc_ua_message,
            'custom_binary': self.handle_custom_binary_message,
            'xml_based': self.handle_xml_message
        }
        
        # Connection management
        self.socket_connection = None
        self.connection_lock = threading.Lock()
        self.message_queue = []
        
        # Publishers and subscribers
        self.ros_publishers = {}
        self.ros_subscribers = {}
        self.setup_ros_interfaces()
        
        # Connection monitoring
        self.connection_timer = self.create_timer(2.0, self.monitor_connection)
        
        # Start legacy protocol handler
        self.start_legacy_communication()
        
        self.get_logger().info('Legacy protocol adapter initialized')
    
    def setup_ros_interfaces(self):
        """Setup ROS interfaces for legacy system bridging"""
        # Setup publishers for data from legacy system
        self.legacy_data_pub = self.create_publisher(String, 'legacy_data', 10)
        
        # Setup subscribers for commands to legacy system
        self.legacy_cmd_sub = self.create_subscription(
            String,
            'legacy_command',
            self.legacy_command_callback,
            10
        )
        
        # Setup diagnostic publisher
        self.diag_pub = self.create_publisher(String, 'legacy_diagnostics', 10)
    
    def start_legacy_communication(self):
        """Start communication with legacy system"""
        def legacy_communication_worker():
            while rclpy.ok():
                try:
                    self.connect_to_legacy_system()
                    
                    while rclpy.ok() and self.is_connected():
                        try:
                            # Receive data from legacy system
                            raw_data = self.receive_from_legacy()
                            
                            if raw_data:
                                # Parse and convert to ROS message
                                ros_msg = self.parse_legacy_data(raw_data)
                                
                                if ros_msg:
                                    self.legacy_data_pub.publish(ros_msg)
                                    
                                    # Process any queued commands
                                    self.process_command_queue()
                            
                            time.sleep(0.01)  # 100 Hz
                        
                        except ConnectionError:
                            self.get_logger().warn('Legacy connection lost, reconnecting...')
                            self.disconnect_from_legacy()
                            time.sleep(1.0)
                        
                        except Exception as e:
                            self.get_logger().error(f'Error in legacy communication: {str(e)}')
                            time.sleep(0.1)
                
                except Exception as e:
                    self.get_logger().error(f'Legacy communication thread error: {str(e)}')
                    time.sleep(2.0)
        
        self.comm_thread = threading.Thread(target=legacy_communication_worker)
        self.comm_thread.daemon = True
        self.comm_thread.start()
    
    def connect_to_legacy_system(self):
        """Connect to legacy system"""
        with self.connection_lock:
            if self.socket_connection is not None:
                return  # Already connected
            
            try:
                self.socket_connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.socket_connection.settimeout(self.legacy_config['timeout'])
                self.socket_connection.connect((
                    self.legacy_config['host'], 
                    self.legacy_config['port']
                ))
                
                self.get_logger().info(f'Connected to legacy system at {self.legacy_config["host"]}:{self.legacy_config["port"]}')
                
                # Send initial handshake if needed
                self.send_handshake()
                
            except Exception as e:
                self.get_logger().error(f'Failed to connect to legacy system: {str(e)}')
                if self.socket_connection:
                    self.socket_connection.close()
                    self.socket_connection = None
    
    def disconnect_from_legacy(self):
        """Disconnect from legacy system"""
        with self.connection_lock:
            if self.socket_connection:
                try:
                    self.socket_connection.close()
                except:
                    pass
                finally:
                    self.socket_connection = None
    
    def is_connected(self) -> bool:
        """Check if connected to legacy system"""
        return self.socket_connection is not None
    
    def receive_from_legacy(self) -> Optional[bytes]:
        """Receive data from legacy system"""
        if not self.is_connected():
            return None
        
        try:
            if self.legacy_config['message_format'] == 'length_prefixed':
                # Receive length-prefixed message
                length_bytes = self.socket_connection.recv(4)  # 4-byte length prefix
                if len(length_bytes) != 4:
                    raise ConnectionError("Incomplete length header")
                
                length = struct.unpack('<I', length_bytes)[0]
                data = self.socket_connection.recv(length)
                
                if len(data) != length:
                    raise ConnectionError("Incomplete message")
                
                return data
            
            elif self.legacy_config['message_format'] == 'delimited':
                # Receive line-delimited message
                data = b''
                while rclpy.ok():
                    chunk = self.socket_connection.recv(1)
                    if not chunk:
                        raise ConnectionError("Connection closed")
                    
                    data += chunk
                    if chunk == b'\n':
                        break
                
                return data[:-1]  # Remove newline
            
            else:
                # Default: receive fixed-size chunks
                data = self.socket_connection.recv(1024)
                return data if data else None
        
        except socket.timeout:
            return None  # Timeout is acceptable
        except Exception as e:
            raise ConnectionError(f"Receive error: {str(e)}")
    
    def send_to_legacy(self, data: bytes) -> bool:
        """Send data to legacy system"""
        if not self.is_connected():
            return False
        
        try:
            self.socket_connection.sendall(data)
            return True
        except Exception as e:
            self.get_logger().error(f'Send to legacy failed: {str(e)}')
            return False
    
    def send_handshake(self):
        """Send initial handshake to legacy system"""
        # Example: Send a simple handshake message
        handshake = "HELLO_ROS2_ADAPTER\n".encode('utf-8')
        self.send_to_legacy(handshake)
    
    def legacy_command_callback(self, msg: String):
        """Handle commands from ROS to be sent to legacy system"""
        try:
            # Convert ROS command to legacy format
            legacy_cmd = self.convert_ros_command_to_legacy(msg.data)
            
            # Queue for sending
            with self.connection_lock:
                self.message_queue.append(legacy_cmd)
            
            self.get_logger().debug(f'Queued command to legacy system: {msg.data}')
        
        except Exception as e:
            self.get_logger().error(f'Error processing ROS command: {str(e)}')
    
    def process_command_queue(self):
        """Process queued commands to legacy system"""
        with self.connection_lock:
            if not self.message_queue:
                return
            
            cmd = self.message_queue.pop(0)
        
        success = self.send_to_legacy(cmd)
        if not success:
            # Requeue failed command?
            with self.connection_lock:
                self.message_queue.insert(0, cmd)
    
    def parse_legacy_data(self, raw_data: bytes) -> Optional[String]:
        """Parse legacy data and convert to ROS message"""
        try:
            # Determine message format and parse accordingly
            if self.legacy_config['message_format'] == 'binary':
                return self.parse_binary_format(raw_data)
            elif self.legacy_config['message_format'] == 'xml_based':
                return self.parse_xml_format(raw_data)
            elif self.legacy_config['message_format'] == 'ascii_delimited':
                return self.parse_ascii_format(raw_data)
            else:
                # Default: treat as string
                parsed_data = raw_data.decode('utf-8', errors='ignore')
                msg = String()
                msg.data = f"LEGACY_DATA: {parsed_data}"
                return msg
        
        except Exception as e:
            self.get_logger().error(f'Error parsing legacy data: {str(e)}')
            return None
    
    def parse_binary_format(self, raw_data: bytes) -> Optional[String]:
        """Parse binary format legacy data"""
        try:
            # Example: Parse a simple binary format
            # Assume: [Header(4 bytes)][ID(2 bytes)][Data(Variable)]
            if len(raw_data) < 6:
                return None
            
            header = raw_data[:4].decode('utf-8', errors='ignore')
            msg_id = struct.unpack('<H', raw_data[4:6])[0]
            data = raw_data[6:].decode('utf-8', errors='ignore')
            
            if header == 'ROB1':
                msg = String()
                msg.data = f"BINARY_MSG: ID={msg_id}, DATA={data}"
                return msg
        
        except Exception as e:
            self.get_logger().error(f'Binary format parsing error: {str(e)}')
        
        return None
    
    def parse_xml_format(self, raw_data: bytes) -> Optional[String]:
        """Parse XML format legacy data"""
        try:
            xml_str = raw_data.decode('utf-8')
            root = ET.fromstring(xml_str)
            
            # Extract relevant information
            message_type = root.findtext('MessageType', 'UNKNOWN')
            message_data = root.findtext('Data', '')
            
            msg = String()
            msg.data = f"XML_MSG: TYPE={message_type}, DATA={message_data}"
            return msg
        
        except ET.ParseError as e:
            self.get_logger().error(f'XML parsing error: {str(e)}')
        except Exception as e:
            self.get_logger().error(f'XML format parsing error: {str(e)}')
        
        return None
    
    def parse_ascii_format(self, raw_data: bytes) -> Optional[String]:
        """Parse ASCII delimited format"""
        try:
            ascii_str = raw_data.decode('utf-8').strip()
            msg = String()
            msg.data = f"ASCII_MSG: {ascii_str}"
            return msg
        
        except Exception as e:
            self.get_logger().error(f'ASCII format parsing error: {str(e)}')
        
        return None
    
    def convert_ros_command_to_legacy(self, ros_command: str) -> bytes:
        """Convert ROS command string to legacy format bytes"""
        # This is protocol-specific
        if self.legacy_config['message_format'] == 'binary':
            # Example binary command format
            cmd_header = 'CMD'.encode('utf-8')
            cmd_data = ros_command.encode('utf-8')
            cmd_length = struct.pack('<H', len(cmd_data))
            return cmd_header + cmd_length + cmd_data
        
        elif self.legacy_config['message_format'] == 'ascii_delimited':
            # Example ASCII command with delimiter
            return f"CMD:{ros_command}\n".encode('utf-8')
        
        else:
            # Default: simple string with newline
            return f"{ros_command}\n".encode('utf-8')
    
    def monitor_connection(self):
        """Monitor connection status"""
        if self.is_connected():
            status_msg = String()
            status_msg.data = f"CONNECTED: {self.legacy_config['host']}:{self.legacy_config['port']}"
            self.diag_pub.publish(status_msg)
        else:
            status_msg = String()
            status_msg.data = "DISCONNECTED"
            self.diag_pub.publish(status_msg)

def main():
    rclpy.init()
    
    adapter = LegacyProtocolAdapter()
    
    try:
        rclpy.spin(adapter)
    except KeyboardInterrupt:
        adapter.get_logger().info('Shutting down legacy adapter')
    finally:
        adapter.disconnect_from_legacy()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Conclusion

Advanced communication patterns and bridging techniques in ROS 2 enable sophisticated integration scenarios that were previously difficult or impossible. From simple ROS 1/ROS 2 bridges to complex multi-domain integrations with cloud services and legacy systems, these patterns provide the flexibility needed for modern robotic applications.

The implementation of these patterns requires careful consideration of timing constraints, data integrity, security implications, and fault tolerance. Properly designed bridges can significantly extend the reach and capabilities of robotic systems, enabling new applications and use cases.

These advanced patterns form the foundation for building robust, scalable robotic systems that can operate effectively in complex, heterogeneous environments. As robotic systems continue to evolve, these bridging and communication patterns will become increasingly important for enabling seamless integration across different domains and platforms.

The next chapters will explore advanced topics including parameter management, launch configuration, and complex system architectures that build upon these communication patterns.