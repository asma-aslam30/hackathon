---
sidebar_position: 7
sidebar_label: ROS 2 Tools, Debugging, and CLI Utilities
---

# ROS 2 Tools, Debugging, and CLI Utilities

## Introduction to ROS 2 Tooling Ecosystem

The ROS 2 tooling ecosystem represents a comprehensive suite of command-line tools, graphical interfaces, and debugging utilities that enable developers to effectively manage, monitor, and debug complex robotic systems. These tools have evolved significantly since ROS 1, incorporating modern software development practices and addressing the distributed nature of ROS 2 applications.

ROS 2 tools encompass a wide range of functionalities including package management, communication monitoring, debugging, visualization, and system introspection. The command-line interface (CLI) tools provide powerful scripting capabilities for automation and system management, while graphical tools like RViz2 and rqt provide intuitive interfaces for visualization and real-time monitoring.

The design philosophy of ROS 2 tools emphasizes modularity, extensibility, and distributed operation. Unlike ROS 1's centralized master architecture, ROS 2 tools can operate across distributed systems, making them suitable for complex deployments involving multiple machines, cloud integration, and multi-robot systems.

The tooling ecosystem is built around several core principles:
- **Modularity**: Each tool serves a specific purpose and can be used independently
- **Extensibility**: New tools can be created and integrated seamlessly
- **Cross-platform compatibility**: Tools work consistently across Linux, Windows, and macOS
- **Distributed operation**: Tools can interact with ROS 2 systems running on different machines
- **Secure operation**: Tools support ROS 2's security features and authentication

## Deep Technical Analysis of Core ROS 2 CLI Tools

### Core Command Line Interface (CLI) Tools

The ROS 2 command-line interface provides an extensive set of tools for system management and development. These tools are built using a plugin architecture that allows for easy extension and customization.

#### ros2 command

The `ros2` command serves as the entry point for all ROS 2 command-line operations. It provides access to various subcommands that handle different aspects of ROS 2 system management.

```bash
# Basic ros2 command structure
ros2 [command] [subcommand] [options] [arguments]

# Common commands:
# - ros2 run: Run a node
# - ros2 node: Node management
# - ros2 topic: Topic management
# - ros2 service: Service management
# - ros2 action: Action management
# - ros2 param: Parameter management
# - ros2 pkg: Package management
# - ros2 msg: Message definition management
# - ros2 srv: Service definition management
# - ros2 bag: Bag file management
```

#### ros2 run - Node Execution

The `ros2 run` command provides the primary mechanism for executing ROS 2 nodes. It handles package resolution, executable discovery, and execution environment setup.

```bash
# Basic usage
ros2 run <package_name> <executable_name> [arguments]

# Examples:
ros2 run turtlesim turtlesim_node
ros2 run my_robot_package my_node --ros-args -p param1:=value1 -p param2:=value2 --remap topic1:=topic2

# Advanced usage with multiple parameters and remappings
ros2 run my_robot_controller controller_node \
    --ros-args \
        -p control_frequency:=100.0 \
        -p max_velocity:=2.0 \
        --remap cmd_vel:=robot/cmd_vel \
        --remap odom:=robot/odom \
        --log-level info
```

#### ros2 node - Node Management

Node management tools provide insight into the current node topology and enable operations on individual nodes.

```bash
# List all nodes
ros2 node list

# List nodes with additional information
ros2 node list --include-hidden-nodes

# Get information about a specific node
ros2 node info <node_name>

# Get information about a specific node with interfaces
ros2 node info <node_name> --interfaces

# Example of node info output analysis
# The command shows published topics, subscribed topics, services offered,
# and parameters exposed by the node.
```

```python
# Python script to programmatically manage nodes
import subprocess
import json
from typing import Dict, List, Optional

class ROS2NodeManager:
    def __init__(self):
        self.nodes = {}
        self.node_interfaces = {}
    
    def list_nodes(self, include_hidden: bool = False) -> List[str]:
        """List all ROS 2 nodes"""
        cmd = ['ros2', 'node', 'list']
        if include_hidden:
            cmd.append('--include-hidden-nodes')
        
        try:
            result = subprocess.run(cmd, capture_output=True, text=True, check=True)
            return [node.strip() for node in result.stdout.strip().split('\n') if node.strip()]
        except subprocess.CalledProcessError as e:
            print(f"Error listing nodes: {e}")
            return []
    
    def get_node_info(self, node_name: str) -> Dict[str, any]:
        """Get detailed information about a specific node"""
        cmd = ['ros2', 'node', 'info', node_name]
        
        try:
            result = subprocess.run(cmd, capture_output=True, text=True, check=True)
            return self.parse_node_info(result.stdout)
        except subprocess.CalledProcessError as e:
            print(f"Error getting node info for {node_name}: {e}")
            return {}
    
    def parse_node_info(self, info_str: str) -> Dict[str, any]:
        """Parse node information output"""
        lines = info_str.strip().split('\n')
        node_info = {
            'published_topics': [],
            'subscribed_topics': [],
            'offered_services': [],
            'provided_actions': []
        }
        
        current_section = None
        for line in lines:
            line = line.strip()
            if 'Published topics:' in line:
                current_section = 'published_topics'
            elif 'Subscribed topics:' in line:
                current_section = 'subscribed_topics'
            elif 'Offered services:' in line:
                current_section = 'offered_services'
            elif 'Provided actions:' in line:
                current_section = 'provided_actions'
            elif line.startswith('- ') and current_section:
                topic_info = line[2:]  # Remove '- ' prefix
                node_info[current_section].append(topic_info)
        
        return node_info
    
    def kill_node(self, node_name: str) -> bool:
        """Kill a specific node"""
        # This is a simplified approach - in practice, you'd need to find the process ID
        # Method 1: Find the process and kill it
        try:
            # Get node PIDs using ros2cli or psutil
            import psutil
            
            # Iterate through all ROS 2 nodes to find the matching one
            # Then kill the process
            for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
                try:
                    cmdline = ' '.join(proc.info['cmdline'])
                    if node_name in cmdline and 'rclcpp' in cmdline:
                        proc.kill()
                        return True
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    continue
            
            return False
        except Exception as e:
            print(f"Error killing node {node_name}: {e}")
            return False

# Example usage
def demonstrate_node_management():
    manager = ROS2NodeManager()
    
    # List all nodes
    nodes = manager.list_nodes()
    print(f"Found {len(nodes)} nodes:")
    for node in nodes:
        print(f"  - {node}")
    
    # Get info for the first node (if any)
    if nodes:
        node_info = manager.get_node_info(nodes[0])
        print(f"\nInformation for {nodes[0]}:")
        print(f"  Published topics: {len(node_info.get('published_topics', []))}")
        print(f"  Subscribed topics: {len(node_info.get('subscribed_topics', []))}")
        print(f"  Offered services: {len(node_info.get('offered_services', []))}")

if __name__ == '__main__':
    demonstrate_node_management()
```

#### ros2 topic - Topic Management

Topic management tools provide comprehensive control over the publish-subscribe communication layer of ROS 2.

```bash
# List all topics
ros2 topic list

# List topics with types
ros2 topic list -t

# Echo a topic's messages
ros2 topic echo <topic_name> [message_type]

# Echo with additional options
ros2 topic echo --field data /chatter std_msgs/msg/String
ros2 topic echo --field position.x /joint_states sensor_msgs/msg/JointState
ros2 topic echo --csv /imu/data sensor_msgs/msg/Imu
ros2 topic echo --json /odom nav_msgs/msg/Odometry

# Get topic information
ros2 topic info <topic_name>

# Publish messages to a topic from command line
ros2 topic pub <topic_name> <message_type> '<message_data>'

# Examples of publishing:
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5}, angular: {z: 0.2}}'
ros2 topic pub --rate 1 /chatter std_msgs/msg/String '{data: "Hello ROS 2"}'
```

```python
import subprocess
import json
from typing import Dict, List, Optional
import threading
import time

class ROS2TopicManager:
    def __init__(self):
        self.topics = {}
        self.topic_info_cache = {}
        self.subscriber_threads = {}
    
    def list_topics(self, with_types: bool = False) -> List[str]:
        """List all topics with optional type information"""
        cmd = ['ros2', 'topic', 'list']
        if with_types:
            cmd.append('-t')
        
        try:
            result = subprocess.run(cmd, capture_output=True, text=True, check=True)
            lines = [line.strip() for line in result.stdout.strip().split('\n') if line.strip()]
            
            if with_types:
                # Parse topic:type format
                topics = []
                for line in lines:
                    if ':' in line:
                        topic, topic_type = line.split(':', 1)
                        topics.append({'name': topic.strip(), 'type': topic_type.strip()})
                    else:
                        topics.append({'name': line, 'type': None})
                return topics
            else:
                return [line for line in lines if line]
        except subprocess.CalledProcessError as e:
            print(f"Error listing topics: {e}")
            return []
    
    def get_topic_info(self, topic_name: str) -> Dict[str, any]:
        """Get detailed information about a specific topic"""
        cmd = ['ros2', 'topic', 'info', topic_name]
        
        try:
            result = subprocess.run(cmd, capture_output=True, text=True, check=True)
            return self.parse_topic_info(result.stdout)
        except subprocess.CalledProcessError as e:
            print(f"Error getting topic info for {topic_name}: {e}")
            return {}
    
    def parse_topic_info(self, info_str: str) -> Dict[str, any]:
        """Parse topic information output"""
        lines = info_str.strip().split('\n')
        topic_info = {
            'topic_name': '',
            'type': '',
            'publisher_count': 0,
            'subscription_count': 0,
            'publishers': [],
            'subscriptions': []
        }
        
        current_section = None
        for line in lines:
            line = line.strip()
            if line.startswith('Type:'):
                topic_info['type'] = line.split(':', 1)[1].strip()
            elif line.startswith('Publisher count:'):
                topic_info['publisher_count'] = int(line.split(':')[1].strip())
            elif line.startswith('Subscription count:'):
                topic_info['subscription_count'] = int(line.split(':')[1].strip())
            elif 'Publishers:' in line:
                current_section = 'publishers'
            elif 'Subscriptions:' in line:
                current_section = 'subscriptions'
            elif line.startswith('- ') and current_section:
                full_info = line[2:]  # Remove '- ' prefix
                node_info = self.parse_node_address(full_info)
                topic_info[current_section].append(node_info)
        
        return topic_info
    
    def parse_node_address(self, node_str: str) -> Dict[str, str]:
        """Parse node address information"""
        # Example: "Node [/turtlesim] [/tmp/ros_-turtlesim-15707-24181382168939916428_0]"
        parts = node_str.split()
        if len(parts) >= 2:
            return {
                'node_name': parts[1].strip('[]'),
                'address': parts[2].strip('[]') if len(parts) > 2 else ''
            }
        return {'node_name': node_str, 'address': ''}
    
    def echo_topic(self, topic_name: str, message_type: str = None, callback=None, duration: int = -1):
        """Echo topic messages (non-blocking version)"""
        def echo_worker():
            cmd = ['ros2', 'topic', 'echo', topic_name]
            if message_type:
                cmd.append(message_type)
            
            # Start the echo process
            process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                universal_newlines=True
            )
            
            start_time = time.time()
            
            for line in iter(process.stdout.readline, ''):
                if duration > 0 and (time.time() - start_time) > duration:
                    process.terminate()
                    break
                
                line = line.strip()
                if line and callback:
                    callback(line)
            
            process.stdout.close()
            process.stderr.close()
            process.wait()
        
        # Start echoing in a separate thread
        thread = threading.Thread(target=echo_worker, daemon=True)
        thread.start()
        self.subscriber_threads[topic_name] = thread
        return thread
    
    def publish_to_topic(self, topic_name: str, message_type: str, message_data: str) -> bool:
        """Publish a single message to a topic"""
        try:
            result = subprocess.run([
                'ros2', 'topic', 'pub', '--once',
                topic_name, message_type, message_data
            ], capture_output=True, text=True, check=True)
            return True
        except subprocess.CalledProcessError as e:
            print(f"Error publishing to topic {topic_name}: {e}")
            print(f"Error output: {e.stderr}")
            return False
    
    def publish_periodic(self, topic_name: str, message_type: str, message_data: str, rate: float) -> threading.Thread:
        """Publish messages to a topic at a specified rate"""
        def publisher_worker():
            import time
            interval = 1.0 / rate
            while True:
                try:
                    self.publish_to_topic(topic_name, message_type, message_data)
                    time.sleep(interval)
                except KeyboardInterrupt:
                    break
        
        thread = threading.Thread(target=publisher_worker, daemon=True)
        thread.start()
        return thread

# Example usage
def demonstrate_topic_management():
    import time
    topic_manager = ROS2TopicManager()
    
    # List all topics
    topics = topic_manager.list_topics(with_types=True)
    print(f"Found {len(topics)} topics:")
    for topic in topics[:10]:  # Show first 10 topics
        topic_type = topic['type'] if isinstance(topic, dict) else 'Unknown'
        topic_name = topic['name'] if isinstance(topic, dict) else topic
        print(f"  - {topic_name}: {topic_type}")
    
    # Get info for a specific topic if any exist
    if topics:
        topic_name = topics[0]['name'] if isinstance(topics[0], dict) else topics[0]
        topic_info = topic_manager.get_topic_info(topic_name)
        if topic_info:
            print(f"\nDetailed info for {topic_name}:")
            print(f"  Type: {topic_info.get('type', 'N/A')}")
            print(f"  Publishers: {topic_info.get('publisher_count', 0)}")
            print(f"  Subscriptions: {topic_info.get('subscription_count', 0)}")

if __name__ == '__main__':
    demonstrate_topic_management()
```

#### ros2 service - Service Management

Service management tools allow interaction with the request-reply communication pattern in ROS 2.

```bash
# List all services
ros2 service list

# List services with types
ros2 service list -t

# Get information about a specific service
ros2 service info <service_name>

# Call a service
ros2 service call <service_name> <service_type> '<request_data>'

# Examples:
ros2 service call /spawn turtlesim/srv/Spawn '{x: 2, y: 2, theta: 0, name: ''turtle2''}'
ros2 service call /clear std_srvs/srv/Empty '{}'
```

```python
class ROS2ServiceManager:
    def __init__(self):
        self.services = {}
        self.service_types = {}
    
    def list_services(self, with_types: bool = False) -> List[str]:
        """List all services with optional type information"""
        cmd = ['ros2', 'service', 'list']
        if with_types:
            cmd.append('-t')
        
        try:
            result = subprocess.run(cmd, capture_output=True, text=True, check=True)
            lines = [line.strip() for line in result.stdout.strip().split('\n') if line.strip()]
            
            if with_types:
                # Parse service:type format
                services = []
                for line in lines:
                    if ':' in line:
                        service, service_type = line.split(':', 1)
                        services.append({'name': service.strip(), 'type': service_type.strip()})
                    else:
                        services.append({'name': line, 'type': None})
                return services
            else:
                return [line for line in lines if line]
        except subprocess.CalledProcessError as e:
            print(f"Error listing services: {e}")
            return []
    
    def get_service_info(self, service_name: str) -> Dict[str, any]:
        """Get detailed information about a specific service"""
        cmd = ['ros2', 'service', 'info', service_name]
        
        try:
            result = subprocess.run(cmd, capture_output=True, text=True, check=True)
            return self.parse_service_info(result.stdout)
        except subprocess.CalledProcessError as e:
            print(f"Error getting service info for {service_name}: {e}")
            return {}
    
    def parse_service_info(self, info_str: str) -> Dict[str, any]:
        """Parse service information output"""
        lines = info_str.strip().split('\n')
        service_info = {
            'service_name': '',
            'type': '',
            'endpoints': []
        }
        
        for line in lines:
            line = line.strip()
            if line.startswith('Type:'):
                service_info['type'] = line.split(':', 1)[1].strip()
            elif line.startswith('- '):
                # Parse endpoint information
                endpoint_info = self.parse_endpoint_info(line[2:])
                service_info['endpoints'].append(endpoint_info)
        
        return service_info
    
    def parse_endpoint_info(self, endpoint_str: str) -> Dict[str, str]:
        """Parse endpoint information"""
        # Format: "Node [/turtlesim] [/tmp/ros_-turtlesim-15707-24181382168939916428_0]"
        parts = endpoint_str.split()
        if len(parts) >= 2:
            return {
                'node_name': parts[1].strip('[]'),
                'address': parts[2].strip('[]') if len(parts) > 2 else ''
            }
        return {'node_name': endpoint_str, 'address': ''}
    
    def call_service(self, service_name: str, service_type: str, request_data: str) -> Optional[str]:
        """Call a service with specified request data"""
        try:
            result = subprocess.run([
                'ros2', 'service', 'call',
                service_name, service_type, request_data
            ], capture_output=True, text=True, check=True)
            return result.stdout
        except subprocess.CalledProcessError as e:
            print(f"Error calling service {service_name}: {e}")
            print(f"Error output: {e.stderr}")
            return None
    
    def discover_service_interfaces(self) -> Dict[str, str]:
        """Discover available service types"""
        try:
            result = subprocess.run([
                'ros2', 'interface', 'list', '-t', 'srv'
            ], capture_output=True, text=True, check=True)
            
            services = {}
            for line in result.stdout.strip().split('\n'):
                if '::' in line:  # Contains package and service name
                    services[line.strip()] = line.strip()
            return services
        except subprocess.CalledProcessError as e:
            print(f"Error discovering service interfaces: {e}")
            return {}

def demonstrate_service_management():
    service_manager = ROS2ServiceManager()
    
    # List all services
    services = service_manager.list_services(with_types=True)
    print(f"Found {len(services)} services:")
    for service in services[:10]:  # Show first 10 services
        service_type = service['type'] if isinstance(service, dict) else 'Unknown'
        service_name = service['name'] if isinstance(service, dict) else service
        print(f"  - {service_name}: {service_type}")
    
    # Discover available service types
    available_types = service_manager.discover_service_interfaces()
    print(f"\nAvailable service types: {len(available_types)}")

if __name__ == '__main__':
    demonstrate_service_management()
```

#### ros2 action - Action Communication

Actions provide goal-oriented communication with feedback, making them suitable for long-running operations.

```bash
# List all actions
ros2 action list

# List actions with types
ros2 action list -t

# Get information about a specific action
ros2 action info <action_name>

# Send a goal to an action server
ros2 action send_goal <action_name> <action_type> '<goal_data>'

# Examples:
ros2 action send_goal /fibonacci action_tutorials_interfaces/action/Fibonacci '{order: 10}'
```

#### ros2 param - Parameter Management

Parameter management tools provide comprehensive control over the configuration system in ROS 2.

```bash
# List parameters for a specific node
ros2 param list <node_name>

# Get parameter value
ros2 param get <node_name> <parameter_name>

# Set parameter value
ros2 param set <node_name> <parameter_name> <value>

# Example:
ros2 param set /turtlesim background_r 255
ros2 param get /turtlesim background_r
```

#### ros2 pkg - Package Management

Package management tools help developers work with ROS 2 packages.

```bash
# List all packages
ros2 pkg list

# Find a specific package
ros2 pkg prefix <package_name>

# Show package information
ros2 pkg info <package_name>

# Find executables in a package
ros2 pkg executables <package_name>

# Example:
ros2 pkg info turtlesim
ros2 pkg executables rclcpp
```

## Advanced Debugging Techniques

### System State Monitoring

Effective ROS 2 debugging starts with understanding the current system state. The following commands provide comprehensive system introspection:

```bash
# Overall system inspection
ros2 doctor

# Detailed system diagnostics
ros2 doctor --report

# Check for common issues
ros2 doctor --report --all
```

### Performance Monitoring

ROS 2 provides tools for monitoring system performance:

```bash
# Monitor network traffic
ros2 doctor --report | grep network

# Check individual node performance
ros2 run demo_nodes_cpp listener & 
# Run in another terminal:
ros2 lifecycle set /listener configure
ros2 lifecycle set /listener activate
```

### Launch File Debugging

Launch files can be complex and difficult to debug. Here are techniques to troubleshoot them:

```yaml
# launch/debuggable_launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Enable detailed logging
    return LaunchDescription([
        Node(
            package='my_package',
            executable='my_node',
            name='my_node',
            parameters=[
                {'use_sim_time': False},
                {'debug_level': 'INFO'}
            ],
            # Enable debugging
            arguments=['--ros-args', '--log-level', 'debug'],
            # Remap for debugging
            remappings=[
                ('input_topic', 'debug_input'),
                ('output_topic', 'debug_output')
            ]
        )
    ])
```

## Graphical Debugging Tools

### RViz2 - Advanced Visualization

RViz2 is the primary visualization tool for ROS 2, offering advanced features for debugging and visualization:

```bash
# Launch RViz2 with a specific configuration
rviz2 -d /path/to/my_config.rviz

# Launch RViz2 with custom settings
rviz2 --display-config /path/to/display.rviz --window-width 1200 --window-height 800
```

### rqt - Plugin-based GUI

rqt provides a plugin-based GUI framework for various tools:

```bash
# Launch rqt with default plugins
rqt

# Launch specific rqt plugins
rqt_console      # Console viewer
rqt_plot         # Plotting tool
rqt_graph        # Graph visualization
rqt_bag          # Bag file player
rqt_logger_level # Logger configuration
rqt_reconfigure  # Dynamic reconfigure
```

## Profiling and Performance Analysis

### Memory and CPU Profiling

```python
import cProfile
import pstats
import subprocess
from io import StringIO
import threading
import time

class ROS2Profiler:
    def __init__(self):
        self.profiler = cProfile.Profile()
        self.profiling_results = {}
    
    def profile_ros2_command(self, cmd: list, duration: int = 5) -> str:
        """Profile a ROS 2 command for a specified duration"""
        def run_command():
            subprocess.run(cmd, capture_output=True, text=True)
        
        # Start profiling
        self.profiler.enable()
        
        # Run the command
        thread = threading.Thread(target=run_command)
        thread.start()
        thread.join(timeout=duration)
        
        # Stop profiling
        self.profiler.disable()
        
        # Get results
        s = StringIO()
        ps = pstats.Stats(self.profiler, stream=s)
        ps.sort_stats('cumulative')
        ps.print_stats()
        
        return s.getvalue()
    
    def profile_system_state(self) -> Dict[str, any]:
        """Profile the overall system state"""
        profile_data = {}
        
        # Profile nodes
        profile_data['nodes'] = len(subprocess.run(
            ['ros2', 'node', 'list'], 
            capture_output=True, text=True
        ).stdout.strip().split('\n'))
        
        # Profile topics
        profile_data['topics'] = len(subprocess.run(
            ['ros2', 'topic', 'list'], 
            capture_output=True, text=True
        ).stdout.strip().split('\n'))
        
        # Profile services
        profile_data['services'] = len(subprocess.run(
            ['ros2', 'service', 'list'], 
            capture_output=True, text=True
        ).stdout.strip().split('\n'))
        
        # Profile actions
        profile_data['actions'] = len(subprocess.run(
            ['ros2', 'action', 'list'], 
            capture_output=True, text=True
        ).stdout.strip().split('\n'))
        
        return profile_data

def demonstrate_profiling():
    profiler = ROS2Profiler()
    
    print("System Profiling Results:")
    profile = profiler.profile_system_state()
    for key, value in profile.items():
        print(f"  {key}: {value}")
    
    # Profile a specific command
    print("\nProfiling ros2 topic list...")
    result = profiler.profile_ros2_command(['ros2', 'topic', 'list'])
    print("First 10 lines of profiler output:")
    print('\n'.join(result.split('\n')[:10]))

if __name__ == '__main__':
    demonstrate_profiling()
```

## Advanced Debugging Scripts

### System Health Check Script

```python
#!/usr/bin/env python3
"""
Advanced ROS 2 system health check script
"""

import subprocess
import json
import sys
import time
from datetime import datetime
from typing import Dict, List, Any

class ROS2SystemHealthChecker:
    def __init__(self):
        self.check_results = {}
    
    def run_health_check(self) -> Dict[str, Any]:
        """Run comprehensive health check"""
        print(f"Starting ROS 2 system health check at {datetime.now()}")
        
        results = {
            'timestamp': datetime.now().isoformat(),
            'checks': {
                'ros_version': self.check_ros_version(),
                'nodes': self.check_nodes(),
                'topics': self.check_topics(), 
                'services': self.check_services(),
                'actions': self.check_actions(),
                'parameters': self.check_parameters(),
                'network': self.check_network_connectivity()
            }
        }
        
        # Overall assessment
        results['assessment'] = self.assess_overall_health(results['checks'])
        
        return results
    
    def check_ros_version(self) -> Dict[str, str]:
        """Check the ROS version"""
        try:
            result = subprocess.run(['echo', '$ROS_DISTRO'], 
                                  capture_output=True, text=True, shell=True)
            version = result.stdout.strip() or 'ROS_DISTRO not set'
            return {'status': 'pass' if version != 'ROS_DISTRO not set' else 'fail', 'version': version}
        except Exception as e:
            return {'status': 'error', 'version': 'unknown', 'error': str(e)}
    
    def check_nodes(self) -> Dict[str, Any]:
        """Check node status"""
        try:
            result = subprocess.run(['ros2', 'node', 'list'], 
                                  capture_output=True, text=True, check=True)
            nodes = [n.strip() for n in result.stdout.strip().split('\n') if n.strip()]
            return {'status': 'pass', 'count': len(nodes), 'nodes': nodes}
        except subprocess.CalledProcessError as e:
            return {'status': 'error', 'count': 0, 'error': str(e), 'nodes': []}
    
    def check_topics(self) -> Dict[str, Any]:
        """Check topic status"""
        try:
            result = subprocess.run(['ros2', 'topic', 'list'], 
                                  capture_output=True, text=True, check=True)
            topics = [t.strip() for t in result.stdout.strip().split('\n') if t.strip()]
            return {'status': 'pass', 'count': len(topics), 'topics': topics}
        except subprocess.CalledProcessError as e:
            return {'status': 'error', 'count': 0, 'error': str(e), 'topics': []}
    
    def check_services(self) -> Dict[str, Any]:
        """Check service status"""
        try:
            result = subprocess.run(['ros2', 'service', 'list'], 
                                  capture_output=True, text=True, check=True)
            services = [s.strip() for s in result.stdout.strip().split('\n') if s.strip()]
            return {'status': 'pass', 'count': len(services), 'services': services}
        except subprocess.CalledProcessError as e:
            return {'status': 'error', 'count': 0, 'error': str(e), 'services': []}
    
    def check_actions(self) -> Dict[str, Any]:
        """Check action status"""
        try:
            result = subprocess.run(['ros2', 'action', 'list'], 
                                  capture_output=True, text=True, check=True)
            actions = [a.strip() for a in result.stdout.strip().split('\n') if a.strip()]
            return {'status': 'pass', 'count': len(actions), 'actions': actions}
        except subprocess.CalledProcessError as e:
            return {'status': 'error', 'count': 0, 'error': str(e), 'actions': []}
    
    def check_parameters(self) -> Dict[str, Any]:
        """Check parameter status"""
        try:
            # Get first available node to check parameters
            nodes_result = subprocess.run(['ros2', 'node', 'list'], 
                                        capture_output=True, text=True, check=True)
            nodes = [n.strip() for n in nodes_result.stdout.strip().split('\n') if n.strip()]
            
            if nodes:
                node = nodes[0]
                cmd = ['ros2', 'param', 'list', node]
                result = subprocess.run(cmd, capture_output=True, text=True)
                
                if result.returncode == 0:
                    params = [p.strip() for p in result.stdout.strip().split('\n') if p.strip()]
                    return {'status': 'pass', 'count': len(params), 'params_sample': params[:5]}
                else:
                    return {'status': 'warning', 'count': 0, 'params_sample': [], 'note': f'No parameters for node {node}'}
            else:
                return {'status': 'warning', 'count': 0, 'params_sample': [], 'note': 'No nodes available'}
        except Exception as e:
            return {'status': 'error', 'count': 0, 'error': str(e), 'params_sample': []}
    
    def check_network_connectivity(self) -> Dict[str, str]:
        """Check basic network connectivity"""
        try:
            # Check if DDS is working by trying a simple communication test
            import rclpy
            from std_msgs.msg import String
            
            rclpy.init()
            node = rclpy.create_node('health_check_publisher')
            publisher = node.create_publisher(String, 'health_check_topic', 10)
            
            msg = String()
            msg.data = f'Health check at {time.time()}'
            publisher.publish(msg)
            
            node.destroy_node()
            rclpy.shutdown()
            
            return {'status': 'pass', 'dds_working': True}
        except Exception as e:
            return {'status': 'error', 'dds_working': False, 'error': str(e)}
    
    def assess_overall_health(self, checks: Dict[str, Any]) -> str:
        """Assess overall system health based on individual checks"""
        if checks['ros_version']['status'] != 'pass':
            return 'critical'
        
        errors = 0
        warnings = 0
        
        for check_name, check_result in checks.items():
            if check_result.get('status') == 'error':
                errors += 1
            elif check_result.get('status') == 'warning':
                warnings += 1
        
        if errors > 0:
            return 'error'
        elif warnings > 0:
            return 'warning'
        else:
            return 'healthy'
    
    def generate_report(self, results: Dict[str, Any]) -> str:
        """Generate a readable report from health check results"""
        report = []
        report.append("ROS 2 System Health Check Report")
        report.append("=" * 40)
        report.append(f"Timestamp: {results['timestamp']}")
        report.append(f"Overall Assessment: {results['assessment'].upper()}")
        report.append("")
        
        for check_name, check_result in results['checks'].items():
            report.append(f"{check_name.upper()}:")
            report.append(f"  Status: {check_result.get('status', 'unknown').upper()}")
            report.append(f"  Count: {check_result.get('count', 'N/A')}")
            
            if check_result.get('error'):
                report.append(f"  Error: {check_result['error']}")
            if check_result.get('warning'):
                report.append(f"  Warning: {check_result['warning']}")
            
            report.append("")
        
        return "\n".join(report)

def main():
    checker = ROS2SystemHealthChecker()
    
    try:
        results = checker.run_health_check()
        report = checker.generate_report(results)
        print(report)
        
        # Exit with appropriate code based on assessment
        if results['assessment'] == 'error':
            sys.exit(1)
        elif results['assessment'] == 'critical':
            sys.exit(2)
    except KeyboardInterrupt:
        print("\nHealth check interrupted by user")
        sys.exit(0)
    except Exception as e:
        print(f"Error during health check: {e}")
        sys.exit(3)

if __name__ == '__main__':
    main()
```

## Containerized Debugging

### Docker-based Debugging Environment

For complex debugging scenarios, containerized environments provide isolated, reproducible debugging setups:

```dockerfile
# Dockerfile for ROS 2 debugging environment
FROM ros:humble

# Install debugging tools
RUN apt-get update && apt-get install -y \
    gdb \
    valgrind \
    htop \
    iotop \
    strace \
    tcpdump \
    && rm -rf /var/lib/apt/lists/*

# Install Python debugging tools
RUN pip3 install --upgrade pip && \
    pip3 install \
    py-spy \
    memory-profiler \
    objgraph

# Copy debug utilities
COPY debug_utils /opt/debug_utils/
RUN chmod +x /opt/debug_utils/*

# Set up debugging environment
ENV ROS_LOG_DIR=/var/log/ros2_debug
RUN mkdir -p $ROS_LOG_DIR

# Default command
CMD ["/bin/bash"]
```

## Best Practices for ROS 2 Debugging

### 1. Systematic Approach

- Start with system-level checks (nodes, topics)
- Progress to component-level analysis
- Use structured logging
- Implement parameter validation

### 2. Performance Considerations

- Use appropriate QoS settings for debugging
- Monitor system resources during debugging
- Minimize logging overhead in production
- Profile before and after implementing fixes

### 3. Security Awareness

- Secure debugging tools in production environments
- Use encrypted communication for remote debugging
- Minimize attack surface during debugging
- Remove debugging artifacts from production builds

## Conclusion

The ROS 2 tooling ecosystem provides comprehensive capabilities for system management, debugging, and performance analysis. The command-line tools offer powerful scripting capabilities, while graphical tools provide intuitive interfaces for real-time monitoring and visualization.

Understanding and utilizing these tools effectively is essential for developing robust, maintainable robotic applications. The modular architecture of the tooling system allows for customized debugging workflows tailored to specific application needs.

The advanced debugging techniques and tools outlined in this chapter provide the foundation for diagnosing complex issues in multi-robot systems, distributed architectures, and real-time applications. Proper utilization of these tools significantly reduces development time and improves the reliability of robotic systems.

These tools form the backbone of the ROS 2 development workflow, enabling developers to build, test, and deploy sophisticated robotic applications with confidence in their correctness and performance.