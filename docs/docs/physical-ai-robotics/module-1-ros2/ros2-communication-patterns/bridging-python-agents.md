---
sidebar_position: 5
sidebar_label: Bridging Python Agents to ROS Controllers with Context7 Integration
---

# Bridging Python Agents to ROS Controllers with Context7 Integration - A Comprehensive Guide

## Table of Contents
1. [Introduction](#introduction)
2. [Deep Technical Analysis](#deep-technical-analysis)
3. [Python Agent Architecture](#python-agent-architecture)
4. [ROS Controller Integration](#ros-controller-integration)
5. [Bridging Mechanisms](#bridging-mechanisms)
6. [Context7 Integration for Documentation](#context7-integration-for-documentation)
7. [Advanced Integration Patterns](#advanced-integration-patterns)
8. [Real-World Examples](#real-world-examples)
9. [Best Practices](#best-practices)
10. [Performance Optimization](#performance-optimization)
11. [Security Considerations](#security-considerations)
12. [Future Developments](#future-developments)
13. [Summary](#summary)

## Introduction

The integration of Python-based intelligent agents with ROS (Robot Operating System) controllers represents a critical advancement in modern robotics, enabling sophisticated AI capabilities to be seamlessly incorporated into robotic systems. This bridging mechanism allows Python agents—often implementing machine learning models, planning algorithms, or high-level decision-making systems—to communicate effectively with low-level ROS controllers that handle actuator commands, sensor processing, and real-time control tasks.

In contemporary robotics applications, especially in the context of humanoid robots and advanced autonomous systems, the ability to bridge high-level cognitive capabilities with low-level control systems is essential. Python, with its rich ecosystem of AI and machine learning libraries, serves as an ideal platform for implementing intelligent agents. Meanwhile, ROS provides the communication infrastructure and control framework that connects various components of a robotic system.

The integration of Context7 documentation systems adds another layer of sophistication to this bridging process, providing on-demand access to up-to-date documentation, API references, and best practices that can enhance both development and runtime capabilities of the integrated system.

This comprehensive guide explores the latest developments in bridging Python agents to ROS controllers as of 2025, including advanced integration patterns, performance optimization techniques, and security considerations. We'll examine how Context7 integration can enhance the development, debugging, and maintenance of these bridged systems, providing developers with immediate access to relevant documentation and best practices.

## Deep Technical Analysis

### The Need for Agent-Controller Integration

Modern robotics applications, particularly those involving humanoid robots and complex autonomous systems, require a multi-layered control architecture. At the highest level, intelligent agents make strategic decisions based on sensor data, goals, and environmental information. At the lowest level, real-time controllers manage actuators, sensors, and basic control loops with precise timing requirements.

The bridging of Python agents to ROS controllers addresses several critical technical challenges:

1. **Abstraction Layering**: Separating high-level decision making from low-level control
2. **Programming Language Integration**: Combining Python's AI/ML capabilities with C++'s real-time performance
3. **Communication Pattern Matching**: Ensuring appropriate communication patterns between different system layers
4. **Timing Requirements**: Managing the different timing constraints of AI algorithms versus real-time control
5. **Data Format Translation**: Converting between Python data structures and ROS message formats

### Architectural Patterns

Several architectural patterns have emerged for bridging Python agents to ROS controllers:

1. **Direct Bridge Pattern**: Direct communication between Python agents and ROS controllers through ROS communication primitives
2. **Proxy Pattern**: Intermediary components that translate between agent and controller interfaces
3. **Service-Oriented Pattern**: Using ROS services for synchronous agent-controller communication
4. **Event-Driven Pattern**: Asynchronous communication through ROS topics with event-based processing
5. **Hybrid Pattern**: Combination of multiple patterns for different types of agent-controller interactions

### rclpy vs Traditional Approaches

The introduction of rclpy (ROS Client Library for Python) has significantly simplified the integration of Python agents with ROS systems. Unlike earlier approaches that required separate processes or complex inter-process communication, rclpy allows Python agents to directly participate in the ROS communication graph as first-class nodes.

The technical advantages of this approach include:

- **Reduced Latency**: Direct message passing without inter-process communication overhead
- **Simplified Deployment**: Single-process solutions for coordinated agent-controller systems
- **Enhanced Debugging**: Unified debugging environment for both agent and control logic
- **Resource Efficiency**: Shared memory spaces and reduced system resource usage

### Real-Time Considerations

When bridging Python agents to ROS controllers, real-time performance becomes a critical concern. Python's garbage collection and dynamic nature can introduce non-deterministic delays that may be problematic for time-critical control systems. Modern approaches address this through:

- **Asynchronous Processing**: Using asyncio and non-blocking operations
- **Thread Isolation**: Separating time-critical control from Python agent processing
- **Buffer Management**: Careful management of data buffers to prevent memory allocation delays
- **Optimized Message Handling**: Efficient serialization/deserialization of ROS messages

## Python Agent Architecture

### Design Principles for Intelligent Agents

Python agents designed for robotic systems should follow specific architectural principles to ensure effective integration with ROS controllers:

1. **Modularity**: Agents should be decomposed into specialized modules that handle different aspects of intelligence
2. **State Management**: Clear separation between agent state and ROS system state
3. **Event Handling**: Asynchronous processing of sensor data and system events
4. **Decision Making**: Clear separation between perception, planning, and action selection
5. **Learning Integration**: Support for online and offline learning within the control loop

### Core Agent Components

A well-designed Python agent for ROS integration typically includes these core components:

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import asyncio
import threading
from queue import Queue, Empty
from typing import Dict, Any, Callable, Optional
import json
import time

class IntelligentAgent:
    """
    Base class for an intelligent agent designed to bridge with ROS controllers.
    Provides core functionality for perception, decision making, and action execution.
    """
    
    def __init__(self, name: str, context7_integration: bool = True):
        self.name = name
        self.context7_integration = context7_integration
        
        # Internal state management
        self.state = {}
        self.perception_buffer = {}
        self.decision_queue = Queue()
        self.action_queue = Queue()
        
        # Event callbacks
        self.event_callbacks: Dict[str, Callable] = {}
        
        # Performance monitoring
        self.metrics = {
            'perception_time': [],
            'decision_time': [],
            'action_time': [],
            'total_cycle_time': []
        }
        
        # Timestamps for timing analysis
        self.timestamps = {}
    
    def register_event_callback(self, event_type: str, callback: Callable):
        """Register a callback function for specific events."""
        self.event_callbacks[event_type] = callback
    
    def process_sensor_data(self, sensor_type: str, data: Any):
        """Process incoming sensor data and update internal perception."""
        start_time = time.time()
        
        if sensor_type not in self.perception_buffer:
            self.perception_buffer[sensor_type] = []
        
        self.perception_buffer[sensor_type].append(data)
        
        # Keep only recent data to prevent memory bloat
        if len(self.perception_buffer[sensor_type]) > 100:
            self.perception_buffer[sensor_type] = self.perception_buffer[sensor_type][-50:]
        
        # Record performance metric
        self.metrics['perception_time'].append(time.time() - start_time)
    
    def make_decision(self) -> Optional[Dict[str, Any]]:
        """Make high-level decisions based on current state and perceptions."""
        start_time = time.time()
        
        # This is where the core AI/decision logic would be implemented
        decision = self._execute_decision_logic()
        
        # Record performance metric
        self.metrics['decision_time'].append(time.time() - start_time)
        
        return decision
    
    def _execute_decision_logic(self) -> Optional[Dict[str, Any]]:
        """Execute the actual decision-making algorithm."""
        # In a real implementation, this would contain the core AI logic
        # such as planning, learning, or reasoning algorithms
        
        # For demonstration, return a simple decision
        if self.perception_buffer.get('sensor_data'):
            return {
                'action': 'move_forward',
                'confidence': 0.8,
                'timestamp': time.time()
            }
        return None
    
    def execute_action(self, action: Dict[str, Any]):
        """Execute an action, potentially sending commands to ROS controllers."""
        start_time = time.time()
        
        # This would typically publish to ROS topics or call ROS services
        # For demonstration, we'll log the action
        print(f"Agent {self.name} executing action: {action}")
        
        # Record performance metric
        self.metrics['action_time'].append(time.time() - start_time)
    
    def get_performance_metrics(self) -> Dict[str, Any]:
        """Return performance metrics for analysis."""
        metrics = {}
        for key, values in self.metrics.items():
            if values:
                metrics[f'{key}_avg'] = sum(values) / len(values)
                metrics[f'{key}_count'] = len(values)
            else:
                metrics[f'{key}_avg'] = 0.0
                metrics[f'{key}_count'] = 0
        
        return metrics

class AgentROSNode(Node):
    """
    ROS Node that integrates the intelligent agent with ROS communication.
    """
    
    def __init__(self, agent: IntelligentAgent):
        super().__init__(f'{agent.name}_bridge_node')
        
        self.agent = agent
        
        # ROS communication components
        self.qos_profile = QoSProfile(depth=10)
        
        # Publishers for commands to ROS controllers
        self.command_publisher = self.create_publisher(
            String, 'agent_commands', self.qos_profile
        )
        
        # Subscribers for sensor data from ROS
        self.sensor_subscriptions = {}
        
        # Create sensor subscriptions for common sensor types
        sensor_types = ['laser_scan', 'imu_data', 'camera_image', 'joint_states']
        for sensor_type in sensor_types:
            self.sensor_subscriptions[sensor_type] = self.create_subscription(
                String,  # In practice, these would be the actual message types
                f'sensor/{sensor_type}',
                lambda msg, st=sensor_type: self._sensor_callback(msg, st),
                self.qos_profile
            )
        
        # Timer for agent execution loop
        self.agent_timer = self.create_timer(0.1, self.agent_execution_loop)
        
        # Timer for performance monitoring
        self.metrics_timer = self.create_timer(5.0, self.log_performance_metrics)
        
        self.get_logger().info(f'Agent-ROS Bridge Node for {agent.name} initialized')
    
    def _sensor_callback(self, msg, sensor_type):
        """Handle incoming sensor data from ROS."""
        try:
            # Parse sensor data (in practice, this would be specific to message type)
            sensor_data = json.loads(msg.data)
            self.agent.process_sensor_data(sensor_type, sensor_data)
        except json.JSONDecodeError:
            # Handle raw data if not JSON
            self.agent.process_sensor_data(sensor_type, msg.data)
        except Exception as e:
            self.get_logger().error(f'Error processing {sensor_type} data: {e}')
    
    def agent_execution_loop(self):
        """Main execution loop for the intelligent agent."""
        start_time = self.get_clock().now().nanoseconds
        
        # Make decisions based on current perceptions
        decision = self.agent.make_decision()
        
        if decision:
            # Execute actions
            self.agent.execute_action(decision)
            
            # Publish commands to ROS controllers
            command_msg = String()
            command_msg.data = json.dumps(decision)
            self.command_publisher.publish(command_msg)
        
        # Record cycle time
        end_time = self.get_clock().now().nanoseconds
        cycle_time = (end_time - start_time) / 1e9  # Convert to seconds
        self.agent.metrics['total_cycle_time'].append(cycle_time)
    
    def log_performance_metrics(self):
        """Log performance metrics periodically."""
        metrics = self.agent.get_performance_metrics()
        self.get_logger().info(f'Agent performance metrics: {metrics}')

def main(args=None):
    rclpy.init(args=args)
    
    # Create an intelligent agent
    agent = IntelligentAgent(name='navigation_agent')
    
    # Create the ROS bridge node
    node = AgentROSNode(agent)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Agent-ROS bridge interrupted')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    from std_msgs.msg import String
    main()
```

### Advanced Agent Architectures

Modern Python agents often implement sophisticated architectures that go beyond simple reactive systems:

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from dataclasses import dataclass
from typing import List, Dict, Any, Optional, Callable
import asyncio
import threading
from queue import Queue
import numpy as np
from collections import deque
import time

@dataclass
class AgentState:
    """Represents the current state of the intelligent agent."""
    position: np.ndarray
    orientation: np.ndarray
    velocity: np.ndarray
    goals: List[np.ndarray]
    obstacles: List[np.ndarray]
    battery_level: float
    sensor_data: Dict[str, Any]

@dataclass
class Action:
    """Represents an action that the agent wants to execute."""
    type: str  # 'move', 'sense', 'communicate', 'wait', etc.
    parameters: Dict[str, Any]
    priority: int
    timestamp: float

class AdvancedIntelligentAgent:
    """
    Advanced intelligent agent with sophisticated planning, learning, and reasoning capabilities.
    """
    
    def __init__(self, name: str, config: Dict[str, Any] = None):
        self.name = name
        self.config = config or {}
        
        # Agent state and perception
        self.state = AgentState(
            position=np.array([0.0, 0.0, 0.0]),
            orientation=np.array([0.0, 0.0, 0.0, 1.0]),  # Quaternion
            velocity=np.array([0.0, 0.0, 0.0]),
            goals=[],
            obstacles=[],
            battery_level=100.0,
            sensor_data={}
        )
        
        # Planning and reasoning components
        self.planner = PathPlanner()
        self.reasoner = Reasoner()
        self.learner = OnlineLearner()
        
        # Action and decision queues
        self.action_queue = Queue()
        self.decision_queue = Queue()
        self.high_priority_actions = []
        
        # Memory and learning
        self.episode_memory = deque(maxlen=1000)
        self.long_term_memory = {}
        
        # Performance metrics
        self.metrics = {
            'planning_time': deque(maxlen=100),
            'reasoning_time': deque(maxlen=100),
            'learning_updates': 0,
            'action_success_rate': deque(maxlen=100)
        }
        
        # Callbacks for external integration
        self.callbacks: Dict[str, List[Callable]] = {
            'sensor_update': [],
            'action_complete': [],
            'goal_reached': []
        }
    
    def update_sensor_data(self, sensor_type: str, data: Any):
        """Process new sensor data and update internal state."""
        # Store sensor data
        self.state.sensor_data[sensor_type] = data
        
        # Process sensor data based on type
        if sensor_type == 'laser_scan':
            self._process_laser_scan(data)
        elif sensor_type == 'camera_image':
            self._process_camera_image(data)
        elif sensor_type == 'imu':
            self._process_imu_data(data)
        elif sensor_type == 'joint_states':
            self._process_joint_states(data)
        
        # Trigger sensor update callbacks
        for callback in self.callbacks['sensor_update']:
            try:
                callback(sensor_type, data)
            except Exception as e:
                print(f"Error in sensor update callback: {e}")
    
    def _process_laser_scan(self, scan_data):
        """Process laser scan data to detect obstacles."""
        # In a real implementation, this would use the actual scan data
        # For demonstration, we'll create mock obstacles
        if hasattr(scan_data, 'ranges'):
            ranges = scan_data.ranges
            # Detect obstacles within 1m
            obstacles = []
            for i, range_val in enumerate(ranges):
                if 0 < range_val < 1.0:  # Obstacle within 1m
                    angle = scan_data.angle_min + i * scan_data.angle_increment
                    x = range_val * np.cos(angle)
                    y = range_val * np.sin(angle)
                    obstacles.append(np.array([x, y, 0.0]))
            self.state.obstacles = obstacles
    
    def _process_camera_image(self, image_data):
        """Process camera image data for perception."""
        # In a real implementation, this would run computer vision algorithms
        pass  # Placeholder for image processing
    
    def _process_imu_data(self, imu_data):
        """Process IMU data to update state."""
        # Update orientation and acceleration
        if hasattr(imu_data, 'orientation'):
            self.state.orientation = np.array([
                imu_data.orientation.x,
                imu_data.orientation.y,
                imu_data.orientation.z,
                imu_data.orientation.w
            ])
    
    def _process_joint_states(self, joint_data):
        """Process joint state data."""
        # Update position based on joint states if available
        pass  # Placeholder for joint state processing
    
    def plan_actions(self) -> List[Action]:
        """Generate a sequence of actions based on current state and goals."""
        start_time = time.time()
        
        # Use planner to generate path if goals exist
        if self.state.goals:
            target = self.state.goals[0]  # Simple: go to first goal
            
            # Plan path to target
            path = self.planner.plan_path(self.state.position, target, self.state.obstacles)
            
            if path:
                # Generate movement actions along the path
                actions = []
                for waypoint in path:
                    move_action = Action(
                        type='move_to',
                        parameters={'target': waypoint},
                        priority=5,
                        timestamp=time.time()
                    )
                    actions.append(move_action)
                
                # Record planning time
                self.metrics['planning_time'].append(time.time() - start_time)
                return actions
        
        # Default action if no specific plan
        idle_action = Action(
            type='idle',
            parameters={},
            priority=1,
            timestamp=time.time()
        )
        return [idle_action]
    
    def execute_decision_cycle(self):
        """Execute one complete decision-making cycle."""
        start_time = time.time()
        
        # Plan actions based on current state
        planned_actions = self.plan_actions()
        
        # Reason about the best sequence of actions
        reasoned_actions = self.reasoner.select_best_actions(
            planned_actions, self.state
        )
        
        # Add actions to execution queue based on priority
        for action in reasoned_actions:
            if action.priority > 3:  # High priority actions
                self.high_priority_actions.append(action)
                self.high_priority_actions.sort(key=lambda x: x.priority, reverse=True)
            else:
                self.action_queue.put(action)
        
        # Record reasoning time
        self.metrics['reasoning_time'].append(time.time() - start_time)
    
    def execute_action(self, action: Action) -> bool:
        """Execute a single action and return success status."""
        try:
            if action.type == 'move_to':
                return self._execute_move_to(action.parameters)
            elif action.type == 'rotate_to':
                return self._execute_rotate_to(action.parameters)
            elif action.type == 'sense':
                return self._execute_sense(action.parameters)
            elif action.type == 'idle':
                return True  # Idle action always succeeds
            else:
                print(f"Unknown action type: {action.type}")
                return False
        except Exception as e:
            print(f"Error executing action {action.type}: {e}")
            return False
    
    def _execute_move_to(self, params: Dict[str, Any]) -> bool:
        """Execute move-to action."""
        # In a real implementation, this would send movement commands to ROS controllers
        target = params.get('target')
        if target is not None:
            # Update agent position (simplified for demonstration)
            direction = target - self.state.position
            distance = np.linalg.norm(direction)
            
            if distance > 0.1:  # If not close enough to target
                # Move towards target
                normalized_direction = direction / distance
                movement = normalized_direction * min(0.1, distance)  # Move 0.1m or less
                self.state.position += movement
                return True
            else:
                # Reached target
                return True
        return False
    
    def _execute_rotate_to(self, params: Dict[str, Any]) -> bool:
        """Execute rotate-to action."""
        # Implementation placeholder
        return True
    
    def _execute_sense(self, params: Dict[str, Any]) -> bool:
        """Execute sense action."""
        # Implementation placeholder
        return True
    
    def get_next_action(self) -> Optional[Action]:
        """Get the next action to execute."""
        # Check high priority actions first
        while self.high_priority_actions:
            return self.high_priority_actions.pop(0)  # Pop highest priority
        
        # Check regular queue
        try:
            return self.action_queue.get_nowait()
        except:
            return None

class PathPlanner:
    """Simple path planning component."""
    
    def plan_path(self, start: np.ndarray, goal: np.ndarray, obstacles: List[np.ndarray]) -> List[np.ndarray]:
        """Plan a path from start to goal avoiding obstacles."""
        # Simplified implementation - in reality would use A*, RRT, or other algorithms
        # For demonstration, return direct path with obstacle avoidance
        path = [start]
        
        # Direct path from start to goal
        direction = goal - start
        distance = np.linalg.norm(direction)
        
        if distance > 0:
            steps = int(distance / 0.1)  # 0.1m steps
            step_vector = direction / max(1, steps)
            
            current_pos = start.copy()
            for i in range(steps):
                current_pos += step_vector
                path.append(current_pos.copy())
        
        # Simple obstacle avoidance - would be much more sophisticated in practice
        return path

class Reasoner:
    """Component for reasoning about actions and decisions."""
    
    def select_best_actions(self, possible_actions: List[Action], state: AgentState) -> List[Action]:
        """Select the best actions based on current state and priorities."""
        # In a real implementation, this would use sophisticated reasoning
        # For demonstration, sort by priority
        return sorted(possible_actions, key=lambda x: x.priority, reverse=True)

class OnlineLearner:
    """Component for online learning and adaptation."""
    
    def update_model(self, state: AgentState, action: Action, reward: float):
        """Update learning model based on state-action-reward."""
        # Placeholder for learning algorithm implementation
        pass

def main(args=None):
    rclpy.init(args=args)
    
    # Create advanced intelligent agent
    agent = AdvancedIntelligentAgent(name='advanced_navigation_agent')
    
    # Add some goals for demonstration
    agent.state.goals = [np.array([2.0, 2.0, 0.0]), np.array([3.0, 3.0, 0.0])]
    
    # Create ROS bridge node
    node = AgentROSNode(agent)  # Using the node class from the previous example
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Advanced agent interrupted')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    from std_msgs.msg import String
    main()
```

### Context7 Integration in Agent Architecture

Integrating Context7 documentation access directly into the agent architecture provides significant benefits:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import requests
import json
from typing import Dict, Any, Optional
from dataclasses import dataclass
import threading
import time

@dataclass
class Context7Result:
    """Result from Context7 documentation query."""
    success: bool
    content: Optional[str]
    metadata: Dict[str, Any]
    timestamp: float

class Context7Agent:
    """
    Intelligent agent with integrated Context7 documentation access.
    """
    
    def __init__(self, name: str, context7_endpoint: str = "http://localhost:8080"):
        self.name = name
        self.context7_endpoint = context7_endpoint
        self.context7_cache = {}
        
        # Performance metrics for Context7 access
        self.context7_metrics = {
            'query_count': 0,
            'cache_hits': 0,
            'avg_response_time': [],
            'error_count': 0
        }
        
        # Lock for thread-safe operations
        self.cache_lock = threading.Lock()
    
    def query_documentation(self, topic: str, use_cache: bool = True) -> Optional[Context7Result]:
        """
        Query Context7 for documentation on a specific topic.
        """
        start_time = time.time()
        
        # Check cache first if enabled
        if use_cache:
            with self.cache_lock:
                if topic in self.context7_cache:
                    cached_result = self.context7_cache[topic]
                    if time.time() - cached_result['timestamp'] < 300:  # 5 min TTL
                        self.context7_metrics['cache_hits'] += 1
                        self.context7_metrics['avg_response_time'].append(time.time() - start_time)
                        return Context7Result(
                            success=True,
                            content=cached_result['content'],
                            metadata=cached_result['metadata'],
                            timestamp=cached_result['timestamp']
                        )
        
        # In a real implementation, this would make an MCP call to Context7
        # For demonstration, we'll simulate the call with mock data
        try:
            documentation = self._get_mock_context7_documentation(topic)
            
            if documentation:
                result = Context7Result(
                    success=True,
                    content=documentation['content'],
                    metadata=documentation['metadata'],
                    timestamp=time.time()
                )
                
                # Cache the result
                if use_cache:
                    with self.cache_lock:
                        self.context7_cache[topic] = {
                            'content': documentation['content'],
                            'metadata': documentation['metadata'],
                            'timestamp': time.time()
                        }
                
                # Update metrics
                self.context7_metrics['query_count'] += 1
                self.context7_metrics['avg_response_time'].append(time.time() - start_time)
                
                return result
            else:
                self.context7_metrics['error_count'] += 1
                return Context7Result(
                    success=False,
                    content=None,
                    metadata={'error': f'No documentation found for {topic}'},
                    timestamp=time.time()
                )
        
        except Exception as e:
            self.context7_metrics['error_count'] += 1
            return Context7Result(
                success=False,
                content=None,
                metadata={'error': str(e)},
                timestamp=time.time()
            )
    
    def _get_mock_context7_documentation(self, topic: str) -> Optional[Dict[str, Any]]:
        """
        Mock function to simulate Context7 documentation retrieval.
        In a real implementation, this would connect to the Context7 MCP server.
        """
        mock_docs = {
            "ros2.action_clients": {
                "content": """
# ROS 2 Action Clients

Action clients in ROS 2 provide a way to send goals to action servers and receive feedback during execution. They are ideal for long-running operations that require progress monitoring.

## Key Features:
- Goal request and response handling
- Feedback during execution
- Cancellation capability
- Asynchronous operation support

## Best Practices:
- Always handle goal rejection
- Monitor feedback for progress
- Implement proper timeout management
- Consider reconnection logic for robustness

## Common Use Cases:
- Robot navigation with progress monitoring
- Complex manipulation tasks
- Calibration procedures
                """,
                "metadata": {
                    "topic": "ros2.action_clients",
                    "version": "humble",
                    "last_updated": "2024-12-14",
                    "quality": "high"
                }
            },
            "python.ai_integration": {
                "content": """
# Python AI Integration with ROS 2

Integrating AI models with ROS 2 requires careful consideration of data formats, timing, and resource management.

## Data Format Considerations:
- Convert numpy arrays to ROS message formats efficiently
- Use appropriate message types for your data
- Consider serialization overhead for large datasets

## Timing and Performance:
- Separate AI processing from real-time control loops
- Use thread pools for model inference
- Implement proper buffering for sensor data

## Memory Management:
- Use generators for large datasets
- Implement object pooling for frequent allocations
- Monitor memory usage during model inference
                """,
                "metadata": {
                    "topic": "python.ai_integration",
                    "version": "humble",
                    "last_updated": "2024-12-14",
                    "quality": "high"
                }
            },
            "robot_localization": {
                "content": """
# Robot Localization in ROS 2

Robot localization determines the robot's position and orientation in a known or unknown environment.

## Localization Methods:
- AMCL (Adaptive Monte Carlo Localization) for 2D pose estimation
- EKF (Extended Kalman Filter) for sensor fusion
- UKF (Unscented Kalman Filter) for nonlinear systems

## Configuration Best Practices:
- Tune covariance matrices based on sensor characteristics
- Use appropriate sensor sources for your environment
- Implement loop closure detection for long-term operation

## Performance Optimization:
- Optimize particle counts for your computational resources
- Use appropriate map resolution
- Consider multi-hypothesis tracking for ambiguous situations
                """,
                "metadata": {
                    "topic": "robot_localization",
                    "version": "humble",
                    "last_updated": "2024-12-14",
                    "quality": "high"
                }
            }
        }
        
        return mock_docs.get(topic)
    
    def get_context7_metrics(self) -> Dict[str, Any]:
        """Get Context7 access metrics."""
        metrics = self.context7_metrics.copy()
        if metrics['avg_response_time']:
            avg_time = sum(metrics['avg_response_time']) / len(metrics['avg_response_time'])
            metrics['avg_response_time_ms'] = avg_time * 1000
        return metrics

class Context7AgentROSNode(Node):
    """
    ROS Node that integrates Context7 agent with ROS communication.
    """
    
    def __init__(self, agent: Context7Agent):
        super().__init__(f'{agent.name}_context7_bridge_node')
        
        self.agent = agent
        
        # Publisher for Context7 queries
        self.context7_request_pub = self.create_publisher(
            String, 'context7_request', 10
        )
        
        # Subscriber for Context7 responses
        self.context7_response_sub = self.create_subscription(
            String, 'context7_response', self.context7_response_callback, 10
        )
        
        # Timer for periodic Context7 queries
        self.context7_timer = self.create_timer(10.0, self.periodic_context7_query)
        
        # Timer for metrics reporting
        self.metrics_timer = self.create_timer(30.0, self.report_context7_metrics)
        
        self.get_logger().info(f'Context7 Agent Bridge Node initialized')
    
    def periodic_context7_query(self):
        """Periodically query Context7 for relevant documentation."""
        topics_to_query = [
            "ros2.action_clients",
            "python.ai_integration",
            "robot_localization"
        ]
        
        for topic in topics_to_query:
            result = self.agent.query_documentation(topic)
            if result:
                if result.success:
                    self.get_logger().info(f'Context7 query for {topic}: Success')
                    self.get_logger().debug(f'Doc content length: {len(result.content or "")} chars')
                else:
                    self.get_logger().warn(f'Context7 query for {topic}: Failed - {result.metadata.get("error", "Unknown error")}')
    
    def context7_response_callback(self, msg):
        """Handle responses from external Context7 system."""
        # In a real implementation, this would handle responses from
        # a separate Context7 MCP server
        pass
    
    def report_context7_metrics(self):
        """Report Context7 access metrics."""
        metrics = self.agent.get_context7_metrics()
        self.get_logger().info(f'Context7 metrics: {metrics}')

def main(args=None):
    rclpy.init(args=args)
    
    # Create Context7-integrated agent
    agent = Context7Agent(name='context7_integrated_agent')
    
    # Create ROS bridge node
    node = Context7AgentROSNode(agent)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Context7 agent interrupted')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## ROS Controller Integration

### Understanding ROS Controllers

ROS controllers are specialized nodes that handle low-level control of robotic systems. They typically interface directly with hardware drivers and implement control algorithms that require precise timing and real-time performance. Controllers in ROS 2 are often part of the `ros2_control` framework, which provides a standardized interface for different types of controllers.

The `ros2_control` framework includes:

1. **Hardware Interface**: Abstraction layer for different hardware platforms
2. **Controller Manager**: Runtime management of active controllers
3. **Controllers**: Specific control algorithms (position, velocity, effort, etc.)
4. **Resource Manager**: Resource allocation and conflict resolution
5. **Transmission Interface**: Mapping between hardware and controller commands

### Controller Architecture Patterns

Modern ROS controller implementations follow specific architectural patterns that facilitate integration with Python agents:

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import SwitchController
import threading
import time
from collections import deque
import numpy as np

class BaseController(Node):
    """
    Base class for ROS controllers with standard interfaces.
    """
    
    def __init__(self, controller_name: str):
        super().__init__(f'{controller_name}_node')
        
        self.controller_name = controller_name
        self.joint_names = []
        self.control_mode = 'position'  # position, velocity, effort
        self.is_active = False
        
        # Joint state storage
        self.current_joint_states = {}
        
        # Control buffers
        self.command_buffer = deque(maxlen=100)
        self.state_buffer = deque(maxlen=100)
        
        # Control loop parameters
        self.control_frequency = 100.0  # Hz
        self.control_period = 1.0 / self.control_frequency
        
        # Publishers and subscribers
        self.state_pub = self.create_publisher(
            JointTrajectoryControllerState, 
            f'{controller_name}/state', 
            QoSProfile(depth=10)
        )
        
        self.joint_state_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            QoSProfile(depth=100)
        )
        
        # Timer for control loop
        self.control_timer = self.create_timer(
            1.0/self.control_frequency, 
            self.control_loop
        )
        
        self.get_logger().info(f'Base controller {controller_name} initialized')
    
    def joint_state_callback(self, msg):
        """Handle joint state updates."""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_joint_states[name] = {
                    'position': msg.position[i],
                    'velocity': msg.velocity[i] if i < len(msg.velocity) else 0.0,
                    'effort': msg.effort[i] if i < len(msg.effort) else 0.0
                }
        
        # Add to state buffer
        self.state_buffer.append({
            'timestamp': self.get_clock().now().nanoseconds,
            'states': self.current_joint_states.copy()
        })
    
    def control_loop(self):
        """Main control loop that runs at fixed frequency."""
        if not self.is_active:
            return
        
        # Implement control algorithm here
        self.execute_control_step()
        
        # Publish current state
        self.publish_state()
    
    def execute_control_step(self):
        """Execute one step of the control algorithm."""
        # This should be implemented by subclasses
        pass
    
    def publish_state(self):
        """Publish current controller state."""
        state_msg = JointTrajectoryControllerState()
        state_msg.header.stamp = self.get_clock().now().to_msg()
        state_msg.header.frame_id = "base_link"
        
        # Publish joint names
        state_msg.joint_names = list(self.current_joint_states.keys())
        
        # Publish desired and actual positions
        for joint_name in state_msg.joint_names:
            joint_state = self.current_joint_states[joint_name]
            state_msg.actual.positions.append(joint_state['position'])
            state_msg.actual.velocities.append(joint_state['velocity'])
            state_msg.actual.effort.append(joint_state['effort'])
            
            # For demonstration, desired = actual
            state_msg.desired.positions.append(joint_state['position'])
            state_msg.desired.velocities.append(joint_state['velocity'])
            state_msg.error.positions.append(0.0)
        
        self.state_pub.publish(state_msg)

class PositionController(BaseController):
    """
    Position controller that implements position-based control.
    """
    
    def __init__(self, controller_name: str, joint_names: list):
        super().__init__(controller_name)
        
        self.joint_names = joint_names
        self.target_positions = {name: 0.0 for name in joint_names}
        self.current_positions = {name: 0.0 for name in joint_names}
        
        # PID parameters
        self.kp = {name: 10.0 for name in joint_names}
        self.ki = {name: 0.1 for name in joint_names}
        self.kd = {name: 0.05 for name in joint_names}
        
        # PID state
        self.integral_error = {name: 0.0 for name in joint_names}
        self.previous_error = {name: 0.0 for name in joint_names}
        self.previous_time = self.get_clock().now()
        
        # Command subscriber
        self.command_sub = self.create_subscription(
            JointTrajectory,
            f'{controller_name}/joint_trajectory',
            self.command_callback,
            QoSProfile(depth=10)
        )
        
        # Activate controller
        self.is_active = True
        
        self.get_logger().info(f'Position controller {controller_name} initialized for joints: {joint_names}')
    
    def command_callback(self, msg):
        """Handle trajectory commands."""
        if msg.points:
            # Get the first point in the trajectory (simplified)
            point = msg.points[0]
            
            # Update target positions
            for i, joint_name in enumerate(msg.joint_names):
                if joint_name in self.target_positions and i < len(point.positions):
                    self.target_positions[joint_name] = point.positions[i]
    
    def execute_control_step(self):
        """Execute position control algorithm."""
        current_time = self.get_clock().now()
        dt = (current_time.nanoseconds - self.previous_time.nanoseconds) / 1e9
        
        if dt <= 0:
            return
        
        # Calculate control for each joint
        for joint_name in self.joint_names:
            if joint_name in self.current_joint_states:
                current_pos = self.current_joint_states[joint_name]['position']
                target_pos = self.target_positions[joint_name]
                
                # Calculate error
                error = target_pos - current_pos
                
                # Update integral (with anti-windup)
                self.integral_error[joint_name] += error * dt
                # Limit integral to prevent windup
                self.integral_error[joint_name] = max(min(self.integral_error[joint_name], 10.0), -10.0)
                
                # Calculate derivative
                derivative = (error - self.previous_error[joint_name]) / dt if dt > 0 else 0.0
                
                # Calculate control output
                control_output = (
                    self.kp[joint_name] * error +
                    self.ki[joint_name] * self.integral_error[joint_name] +
                    self.kd[joint_name] * derivative
                )
                
                # Update previous values
                self.previous_error[joint_name] = error
                self.previous_time = current_time

class TrajectoryController(PositionController):
    """
    Trajectory controller that implements smooth trajectory following.
    """
    
    def __init__(self, controller_name: str, joint_names: list):
        super().__init__(controller_name, joint_names)
        
        # Trajectory execution variables
        self.active_trajectory = None
        self.trajectory_progress = 0
        self.trajectory_start_time = None
        
        # Trajectory interpolation
        self.interpolation_method = 'cubic'  # or 'linear'
        
        self.get_logger().info(f'Trajectory controller {controller_name} initialized')
    
    def command_callback(self, msg):
        """Handle trajectory commands with time-based execution."""
        self.active_trajectory = msg
        self.trajectory_progress = 0
        self.trajectory_start_time = self.get_clock().now()
    
    def execute_control_step(self):
        """Execute trajectory following control."""
        if self.active_trajectory and self.trajectory_start_time:
            # Calculate progress along trajectory
            elapsed_time = self.get_clock().now() - self.trajectory_start_time
            elapsed_sec = elapsed_time.nanoseconds / 1e9
            
            # Find appropriate trajectory point
            current_point = self._get_current_trajectory_point(elapsed_sec)
            
            if current_point:
                # Update target positions from trajectory
                for i, joint_name in enumerate(self.active_trajectory.joint_names):
                    if i < len(current_point.positions):
                        self.target_positions[joint_name] = current_point.positions[i]
        
        # Execute base position control
        super().execute_control_step()
    
    def _get_current_trajectory_point(self, elapsed_time: float) -> Optional[JointTrajectoryPoint]:
        """Get the current trajectory point based on elapsed time."""
        if not self.active_trajectory or not self.active_trajectory.points:
            return None
        
        # Find the appropriate point in the trajectory
        for i, point in enumerate(self.active_trajectory.points):
            point_time = point.time_from_start.sec + point.time_from_start.nanosec / 1e9
            
            if elapsed_time <= point_time:
                # Perform interpolation between previous point and current point
                if i > 0:
                    prev_point = self.active_trajectory.points[i-1]
                    prev_time = prev_point.time_from_start.sec + prev_point.time_from_start.nanosec / 1e9
                    
                    # Linear interpolation
                    alpha = (elapsed_time - prev_time) / (point_time - prev_time) if point_time != prev_time else 1.0
                    
                    interpolated_point = JointTrajectoryPoint()
                    interpolated_point.positions = []
                    
                    for j in range(len(point.positions)):
                        pos = prev_point.positions[j] + alpha * (point.positions[j] - prev_point.positions[j])
                        interpolated_point.positions.append(pos)
                    
                    interpolated_point.velocities = []  # Simplified
                    interpolated_point.accelerations = []  # Simplified
                    
                    return interpolated_point
                else:
                    # First point
                    return point
        
        # If past the end of the trajectory, use the last point
        return self.active_trajectory.points[-1]

def main(args=None):
    rclpy.init(args=args)
    
    # Create trajectory controller
    controller = TrajectoryController(
        controller_name='arm_controller',
        joint_names=['joint1', 'joint2', 'joint3']
    )
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Controller interrupted')
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Agent-Controller Communication Protocols

Establishing effective communication between Python agents and ROS controllers requires careful consideration of communication patterns, message formats, and timing requirements:

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String, Float64MultiArray
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import JointState
import json
import time
from typing import Dict, Any, Callable, Optional
import threading
from queue import Queue, Empty

class AgentControllerBridge(Node):
    """
    Bridge between Python agents and ROS controllers with multiple communication protocols.
    """
    
    def __init__(self, agent_name: str, controller_prefix: str):
        super().__init__(f'{agent_name}_controller_bridge')
        
        self.agent_name = agent_name
        self.controller_prefix = controller_prefix
        
        # QoS profiles for different communication needs
        self.reliable_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST
        )
        
        self.best_effort_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )
        
        # Publishers for different controller interfaces
        self.cmd_vel_pub = self.create_publisher(
            Twist, f'{controller_prefix}/cmd_vel', self.reliable_qos
        )
        
        self.joint_cmd_pub = self.create_publisher(
            JointState, f'{controller_prefix}/joint_commands', self.reliable_qos
        )
        
        self.goal_pub = self.create_publisher(
            Pose, f'{controller_prefix}/goal', self.reliable_qos
        )
        
        # Subscribers for controller feedback
        self.odom_sub = self.create_subscription(
            String, f'{controller_prefix}/odometry', self.odom_callback, self.best_effort_qos
        )
        
        self.joint_state_sub = self.create_subscription(
            JointState, f'{controller_prefix}/joint_states', self.joint_state_callback, self.best_effort_qos
        )
        
        # Agent communication interfaces
        self.agent_command_sub = self.create_subscription(
            String, f'{agent_name}/commands', self.agent_command_callback, self.reliable_qos
        )
        
        self.controller_feedback_pub = self.create_publisher(
            String, f'{agent_name}/controller_feedback', self.reliable_qos
        )
        
        # Communication state
        self.current_odometry = {}
        self.current_joint_states = {}
        self.agent_commands = Queue()
        self.controller_status = 'idle'
        
        # Protocol handlers
        self.protocol_handlers = {
            'direct_command': self.handle_direct_command,
            'trajectory_request': self.handle_trajectory_request,
            'status_request': self.handle_status_request,
            'parameter_update': self.handle_parameter_update
        }
        
        # Communication timer
        self.comm_timer = self.create_timer(0.05, self.communication_loop)  # 20 Hz
        
        # Performance monitoring
        self.comm_stats = {
            'messages_sent': 0,
            'messages_received': 0,
            'avg_latency': [],
            'error_count': 0
        }
        
        self.get_logger().info(f'Agent-Controller Bridge initialized for {agent_name}')
    
    def agent_command_callback(self, msg):
        """Handle commands from the Python agent."""
        try:
            command_data = json.loads(msg.data)
            self.agent_commands.put(command_data)
            self.comm_stats['messages_received'] += 1
        except json.JSONDecodeError:
            self.get_logger().error(f'Invalid JSON command: {msg.data}')
            self.comm_stats['error_count'] += 1
        except Exception as e:
            self.get_logger().error(f'Error processing agent command: {e}')
            self.comm_stats['error_count'] += 1
    
    def odom_callback(self, msg):
        """Handle odometry feedback from controller."""
        try:
            self.current_odometry = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f'Invalid odometry data: {msg.data}')
    
    def joint_state_callback(self, msg):
        """Handle joint state feedback from controller."""
        self.current_joint_states = {
            name: {
                'position': msg.position[i] if i < len(msg.position) else 0.0,
                'velocity': msg.velocity[i] if i < len(msg.velocity) else 0.0,
                'effort': msg.effort[i] if i < len(msg.effort) else 0.0
            }
            for i, name in enumerate(msg.name)
        }
    
    def communication_loop(self):
        """Main communication loop for processing agent commands."""
        # Process commands from agent
        while True:
            try:
                command = self.agent_commands.get_nowait()
                self.process_agent_command(command)
            except Empty:
                break  # No more commands to process
    
    def process_agent_command(self, command: Dict[str, Any]):
        """Process commands from the Python agent using appropriate protocol."""
        command_type = command.get('type', 'unknown')
        
        if command_type in self.protocol_handlers:
            start_time = time.time()
            
            try:
                result = self.protocol_handlers[command_type](command)
                
                # Calculate latency and update stats
                latency = time.time() - start_time
                self.comm_stats['avg_latency'].append(latency)
                
                # Publish result back to agent if needed
                if result:
                    response_msg = String()
                    response_msg.data = json.dumps(result)
                    self.controller_feedback_pub.publish(response_msg)
                
            except Exception as e:
                self.get_logger().error(f'Error processing command {command_type}: {e}')
                self.comm_stats['error_count'] += 1
        else:
            self.get_logger().warn(f'Unknown command type: {command_type}')
    
    def handle_direct_command(self, command: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """Handle direct velocity or position commands."""
        cmd_type = command.get('command', 'unknown')
        
        if cmd_type == 'cmd_vel':
            # Send velocity command to controller
            twist_msg = Twist()
            twist_msg.linear.x = command.get('linear_x', 0.0)
            twist_msg.linear.y = command.get('linear_y', 0.0)
            twist_msg.linear.z = command.get('linear_z', 0.0)
            twist_msg.angular.x = command.get('angular_x', 0.0)
            twist_msg.angular.y = command.get('angular_y', 0.0)
            twist_msg.angular.z = command.get('angular_z', 0.0)
            
            self.cmd_vel_pub.publish(twist_msg)
            self.comm_stats['messages_sent'] += 1
            
            return {'status': 'command_sent', 'command': 'cmd_vel'}
        
        elif cmd_type == 'joint_cmd':
            # Send joint commands to controller
            joint_msg = JointState()
            joint_msg.header.stamp = self.get_clock().now().to_msg()
            joint_msg.name = command.get('joint_names', [])
            joint_msg.position = command.get('positions', [])
            joint_msg.velocity = command.get('velocities', [])
            joint_msg.effort = command.get('efforts', [])
            
            self.joint_cmd_pub.publish(joint_msg)
            self.comm_stats['messages_sent'] += 1
            
            return {'status': 'command_sent', 'command': 'joint_cmd'}
        
        return {'status': 'unknown_command', 'command': cmd_type}
    
    def handle_trajectory_request(self, command: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """Handle trajectory planning and execution requests."""
        # For demonstration, send a simple goal
        if 'goal' in command:
            goal = command['goal']
            pose_msg = Pose()
            pose_msg.position.x = goal.get('x', 0.0)
            pose_msg.position.y = goal.get('y', 0.0)
            pose_msg.position.z = goal.get('z', 0.0)
            pose_msg.orientation.w = 1.0  # Simplified orientation
            
            self.goal_pub.publish(pose_msg)
            self.comm_stats['messages_sent'] += 1
            
            return {'status': 'trajectory_sent', 'goal': goal}
        
        return {'status': 'invalid_trajectory', 'error': 'No goal specified'}
    
    def handle_status_request(self, command: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """Handle status requests from the agent."""
        return {
            'status': self.controller_status,
            'timestamp': self.get_clock().now().nanoseconds,
            'joint_states': self.current_joint_states,
            'odometry': self.current_odometry
        }
    
    def handle_parameter_update(self, command: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """Handle parameter updates for controller tuning."""
        # In a real implementation, this would update controller parameters
        # For demonstration, just acknowledge the update
        parameters = command.get('parameters', {})
        
        self.get_logger().info(f'Parameter update received: {list(parameters.keys())}')
        
        return {
            'status': 'parameters_updated',
            'updated_params': list(parameters.keys())
        }

def main(args=None):
    rclpy.init(args=args)
    
    # Create bridge between agent and controller
    bridge = AgentControllerBridge(
        agent_name='navigation_agent',
        controller_prefix='mobile_base_controller'
    )
    
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        bridge.get_logger().info('Agent-Controller Bridge interrupted')
    finally:
        bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Bridging Mechanisms

### Direct Integration Approaches

The most straightforward approach to bridging Python agents to ROS controllers is direct integration within the same node process:

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import LaserScan, Imu, JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import numpy as np
import time
import threading
from queue import Queue
import json

class DirectAgentControllerNode(Node):
    """
    Direct integration approach: Python agent and ROS controllers in same node.
    This provides the lowest latency communication but requires careful resource management.
    """
    
    def __init__(self):
        super().__init__('direct_agent_controller_node')
        
        # Agent components
        self.agent_state = {
            'position': np.array([0.0, 0.0, 0.0]),
            'orientation': np.array([0.0, 0.0, 0.0, 1.0]),
            'velocity': np.array([0.0, 0.0, 0.0]),
            'goals': [],
            'obstacles': [],
            'battery_level': 100.0,
            'is_moving': False
        }
        
        self.agent_memory = {
            'visited_positions': [],
            'explored_areas': [],
            'learned_patterns': {}
        }
        
        # Controller components
        self.controller_state = {
            'joint_positions': {},
            'joint_velocities': {},
            'current_mode': 'idle',
            'last_command_time': 0.0,
            'safety_limits': {
                'max_velocity': 1.0,
                'max_acceleration': 2.0,
                'collision_threshold': 0.5
            }
        }
        
        # Communication setup
        qos_profile = QoSProfile(depth=10)
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos_profile)
        self.joint_cmd_pub = self.create_publisher(JointState, 'joint_commands', qos_profile)
        self.status_pub = self.create_publisher(String, 'agent_status', qos_profile)
        
        # Subscribers
        self.laser_sub = self.create_subscription(LaserScan, 'scan', self.laser_callback, qos_profile)
        self.imu_sub = self.create_subscription(Imu, 'imu/data', self.imu_callback, qos_profile)
        self.joint_state_sub = self.create_subscription(JointState, 'joint_states', self.joint_state_callback, qos_profile)
        
        # Agent control timer
        self.agent_timer = self.create_timer(0.1, self.agent_control_loop)  # 10 Hz
        
        # Controller execution timer
        self.controller_timer = self.create_timer(0.01, self.controller_execution_loop)  # 100 Hz
        
        # Performance monitoring
        self.performance_stats = {
            'agent_cycles': 0,
            'controller_cycles': 0,
            'avg_agent_time': [],
            'avg_controller_time': [],
            'sensor_updates': 0
        }
        
        self.get_logger().info('Direct Agent-Controller Integration Node initialized')
    
    def laser_callback(self, msg):
        """Handle laser scan data for agent perception."""
        # Process laser data for obstacle detection
        ranges = [r for r in msg.ranges if not (np.isnan(r) or np.isinf(r)) and r > 0]
        
        if ranges:
            min_range = min(ranges)
            self.agent_state['obstacles'] = [{'distance': min_range, 'direction': 'front'}]
            self.controller_state['safety_limits']['collision_threshold'] = min_range
        
        self.performance_stats['sensor_updates'] += 1
    
    def imu_callback(self, msg):
        """Handle IMU data for agent localization."""
        self.agent_state['orientation'] = np.array([
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ])
        
        self.agent_state['velocity'] = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])
    
    def joint_state_callback(self, msg):
        """Handle joint state feedback from controllers."""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.controller_state['joint_positions'][name] = msg.position[i]
            if i < len(msg.velocity):
                self.controller_state['joint_velocities'][name] = msg.velocity[i]
    
    def agent_control_loop(self):
        """Main agent control loop with decision making."""
        start_time = time.time()
        
        # Update agent state based on current information
        self.update_agent_state()
        
        # Make high-level decisions
        action = self.make_decision()
        
        if action:
            # Execute action through controller interface
            self.execute_action(action)
        
        # Publish status
        self.publish_status()
        
        # Update performance metrics
        agent_time = time.time() - start_time
        self.performance_stats['avg_agent_time'].append(agent_time)
        self.performance_stats['agent_cycles'] += 1
    
    def controller_execution_loop(self):
        """Real-time controller execution loop."""
        start_time = time.time()
        
        # Execute low-level control based on agent commands
        self.execute_low_level_control()
        
        # Update controller state
        self.update_controller_state()
        
        # Update performance metrics
        controller_time = time.time() - start_time
        self.performance_stats['avg_controller_time'].append(controller_time)
        self.performance_stats['controller_cycles'] += 1
    
    def update_agent_state(self):
        """Update agent's internal state based on sensor data."""
        # This would include complex perception and localization algorithms
        # For demonstration, we'll do a simple update
        pass
    
    def make_decision(self):
        """Make high-level decisions based on current state."""
        # Simple decision making for demonstration
        if self.agent_state['obstacles']:
            obstacle = self.agent_state['obstacles'][0]
            if obstacle['distance'] < 0.5:  # Emergency stop
                return {
                    'type': 'emergency_stop',
                    'reason': 'obstacle_too_close',
                    'distance': obstacle['distance']
                }
        
        # Move forward if clear
        return {
            'type': 'move',
            'linear_velocity': 0.2,
            'angular_velocity': 0.0
        }
    
    def execute_action(self, action):
        """Execute agent action through controller interface."""
        if action['type'] == 'move':
            twist_msg = Twist()
            twist_msg.linear.x = action.get('linear_velocity', 0.0)
            twist_msg.angular.z = action.get('angular_velocity', 0.0)
            
            self.cmd_vel_pub.publish(twist_msg)
            self.agent_state['is_moving'] = True
            self.controller_state['last_command_time'] = time.time()
        
        elif action['type'] == 'emergency_stop':
            # Emergency stop - zero velocities
            stop_msg = Twist()
            self.cmd_vel_pub.publish(stop_msg)
            self.agent_state['is_moving'] = False
    
    def execute_low_level_control(self):
        """Execute low-level control algorithms."""
        # This runs at high frequency for time-critical control
        current_time = time.time()
        
        # Check for safety timeouts
        if (current_time - self.controller_state['last_command_time']) > 2.0:  # 2 second timeout
            # Emergency stop if no recent commands
            stop_msg = Twist()
            self.cmd_vel_pub.publish(stop_msg)
    
    def update_controller_state(self):
        """Update controller-specific state."""
        # Update any controller-specific state variables
        pass
    
    def publish_status(self):
        """Publish agent status to monitoring systems."""
        status_msg = String()
        status_msg.data = json.dumps({
            'position': self.agent_state['position'].tolist(),
            'is_moving': self.agent_state['is_moving'],
            'obstacle_distance': self.agent_state['obstacles'][0]['distance'] if self.agent_state['obstacles'] else float('inf'),
            'battery_level': self.agent_state['battery_level'],
            'timestamp': time.time()
        })
        
        self.status_pub.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DirectAgentControllerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Direct integration node interrupted')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Message Passing Integration

For more complex architectures, separate processes with message passing can provide better isolation and scalability:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan, JointState
from geometry_msgs.msg import Twist
import multiprocessing
import threading
import queue
import json
import time
import asyncio

class MessagePassingAgent:
    """
    Python agent running in a separate process that communicates via message queues.
    """
    
    def __init__(self, command_queue, data_queue, result_queue):
        self.command_queue = command_queue
        self.data_queue = data_queue
        self.result_queue = result_queue
        
        # Agent state
        self.state = {
            'position': [0.0, 0.0, 0.0],
            'goals': [],
            'obstacles': []
        }
        
        self.running = True
    
    def run(self):
        """Main execution loop for the agent."""
        while self.running:
            # Check for commands
            try:
                if not self.command_queue.empty():
                    command = self.command_queue.get_nowait()
                    self.handle_command(command)
            except queue.Empty:
                pass
            
            # Check for sensor data
            try:
                if not self.data_queue.empty():
                    sensor_data = self.data_queue.get_nowait()
                    self.process_sensor_data(sensor_data)
            except queue.Empty:
                pass
            
            # Make decisions
            action = self.make_decision()
            if action:
                self.result_queue.put(action)
            
            # Sleep briefly to prevent busy waiting
            time.sleep(0.01)
    
    def handle_command(self, command):
        """Handle commands from ROS system."""
        if command['type'] == 'set_goal':
            self.state['goals'].append(command['goal'])
        elif command['type'] == 'stop':
            self.running = False
    
    def process_sensor_data(self, sensor_data):
        """Process incoming sensor data."""
        if sensor_data['type'] == 'laser_scan':
            # Process laser scan for obstacles
            ranges = sensor_data['data']['ranges']
            valid_ranges = [r for r in ranges if 0 < r < 5.0]  # Only valid ranges up to 5m
            if valid_ranges:
                min_range = min(valid_ranges)
                self.state['obstacles'] = [{'distance': min_range, 'direction': 'front'}]
    
    def make_decision(self):
        """Make high-level decisions."""
        if self.state['obstacles']:
            obstacle = self.state['obstacles'][0]
            if obstacle['distance'] < 0.5:  # Emergency stop
                return {
                    'type': 'command',
                    'command': 'emergency_stop',
                    'reason': 'obstacle_detected'
                }
        
        # Continue with navigation
        return {
            'type': 'command',
            'command': 'move_forward',
            'velocity': 0.2
        }

class MessagePassingBridgeNode(Node):
    """
    ROS Node that bridges to a separate Python agent process using message queues.
    """
    
    def __init__(self):
        super().__init__('message_passing_bridge_node')
        
        # Create message queues for communication
        self.command_queue = multiprocessing.Queue()
        self.data_queue = multiprocessing.Queue()
        self.result_queue = multiprocessing.Queue()
        
        # Start the agent process
        self.agent_process = multiprocessing.Process(
            target=self.run_agent_process,
            args=(self.command_queue, self.data_queue, self.result_queue)
        )
        self.agent_process.start()
        
        # ROS communication
        self.qos_profile = 10
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', self.qos_profile)
        self.status_pub = self.create_publisher(String, 'agent_status', self.qos_profile)
        
        # Subscribers
        self.laser_sub = self.create_subscription(
            LaserScan, 'scan', self.laser_callback, self.qos_profile
        )
        self.joint_state_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, self.qos_profile
        )
        
        # Timer for checking agent results
        self.result_check_timer = self.create_timer(0.05, self.check_agent_results)
        
        # Timer for sending data to agent
        self.data_send_timer = self.create_timer(0.1, self.send_sensor_data)
        
        self.get_logger().info('Message Passing Bridge Node initialized')
    
    def run_agent_process(self, command_queue, data_queue, result_queue):
        """Run the agent in a separate process."""
        agent = MessagePassingAgent(command_queue, data_queue, result_queue)
        agent.run()
    
    def laser_callback(self, msg):
        """Handle laser scan data."""
        # Convert ROS message to a format suitable for the agent
        laser_data = {
            'type': 'laser_scan',
            'data': {
                'ranges': list(msg.ranges),
                'angle_min': msg.angle_min,
                'angle_max': msg.angle_max,
                'angle_increment': msg.angle_increment
            }
        }
        
        # Put data in queue for agent
        self.data_queue.put(laser_data)
    
    def joint_state_callback(self, msg):
        """Handle joint state data."""
        joint_data = {
            'type': 'joint_state',
            'data': {
                'names': list(msg.name),
                'positions': list(msg.position),
                'velocities': list(msg.velocity)
            }
        }
        
        self.data_queue.put(joint_data)
    
    def check_agent_results(self):
        """Check for results from the agent process."""
        try:
            if not self.result_queue.empty():
                result = self.result_queue.get_nowait()
                self.handle_agent_result(result)
        except queue.Empty:
            pass
    
    def handle_agent_result(self, result):
        """Handle results from the agent."""
        if result['type'] == 'command':
            command = result['command']
            
            if command == 'emergency_stop':
                # Stop the robot
                stop_msg = Twist()
                self.cmd_vel_pub.publish(stop_msg)
                self.get_logger().warn('Emergency stop command received from agent')
            
            elif command == 'move_forward':
                # Move forward
                velocity = result.get('velocity', 0.2)
                cmd_msg = Twist()
                cmd_msg.linear.x = velocity
                cmd_msg.angular.z = 0.0
                self.cmd_vel_pub.publish(cmd_msg)
    
    def send_sensor_data(self):
        """Send periodic data to the agent (if needed)."""
        # This can be used to send periodic updates to the agent
        # For example, position updates if GPS or odometry is available
        pass

def main(args=None):
    rclpy.init(args=args)
    node = MessagePassingBridgeNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Message passing bridge interrupted')
        # Terminate the agent process
        node.command_queue.put({'type': 'stop'})
        node.agent_process.join(timeout=2.0)
        if node.agent_process.is_alive():
            node.agent_process.terminate()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Context7 Integration in Bridging Mechanisms

Integrating Context7 documentation access into bridging mechanisms allows for dynamic optimization and debugging:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import time
import json
from typing import Dict, Any, Optional

class Context7BridgingNode(Node):
    """
    Bridging node with integrated Context7 documentation access.
    """
    
    def __init__(self):
        super().__init__('context7_bridging_node')
        
        # Context7 integration components
        self.context7_cache = {}
        self.context7_request_count = 0
        self.context7_cache_lock = threading.Lock()
        
        # Bridging components
        self.qos_profile = 10
        
        # Publishers and subscribers
        self.agent_cmd_pub = self.create_publisher(String, 'agent_commands', self.qos_profile)
        self.controller_cmd_pub = self.create_publisher(String, 'controller_commands', self.qos_profile)
        
        self.agent_response_sub = self.create_subscription(
            String, 'agent_responses', self.agent_response_callback, self.qos_profile
        )
        self.controller_response_sub = self.create_subscription(
            String, 'controller_responses', self.controller_response_callback, self.qos_profile
        )
        
        # Context7 interface
        self.context7_request_pub = self.create_publisher(String, 'context7_requests', self.qos_profile)
        self.context7_response_sub = self.create_subscription(
            String, 'context7_responses', self.context7_response_callback, self.qos_profile
        )
        
        # Timer for periodic Context7 queries
        self.context7_timer = self.create_timer(5.0, self.periodic_context7_query)
        
        # Timer for bridging operations
        self.bridge_timer = self.create_timer(0.1, self.bridge_operations)
        
        self.get_logger().info('Context7 Bridging Node initialized')
    
    def agent_response_callback(self, msg):
        """Handle responses from the Python agent."""
        try:
            response_data = json.loads(msg.data)
            
            # Check if agent needs documentation help
            if response_data.get('needs_documentation'):
                topic = response_data['needs_documentation']
                self.request_context7_documentation(topic)
            
            # Process the agent's decision
            self.process_agent_decision(response_data)
            
        except json.JSONDecodeError:
            self.get_logger().error(f'Invalid JSON from agent: {msg.data}')
    
    def controller_response_callback(self, msg):
        """Handle responses from the ROS controller."""
        try:
            response_data = json.loads(msg.data)
            
            # Process controller feedback
            self.process_controller_feedback(response_data)
            
            # Check if controller needs documentation
            if response_data.get('documentation_needed'):
                topic = response_data['documentation_needed']
                self.request_context7_documentation(topic)
                
        except json.JSONDecodeError:
            self.get_logger().error(f'Invalid JSON from controller: {msg.data}')
    
    def context7_response_callback(self, msg):
        """Handle responses from Context7 documentation system."""
        try:
            response_data = json.loads(msg.data)
            topic = response_data.get('topic')
            content = response_data.get('content')
            
            if topic and content:
                with self.context7_cache_lock:
                    self.context7_cache[topic] = {
                        'content': content,
                        'timestamp': time.time(),
                        'retrieved_from': response_data.get('source', 'unknown')
                    }
                
                self.get_logger().info(f'Context7 documentation cached for: {topic}')
                
        except json.JSONDecodeError:
            self.get_logger().error(f'Invalid Context7 response: {msg.data}')
    
    def request_context7_documentation(self, topic: str):
        """Request documentation from Context7 system."""
        request_msg = String()
        request_data = {
            'request_type': 'documentation',
            'topic': topic,
            'requester': self.get_name(),
            'timestamp': time.time()
        }
        request_msg.data = json.dumps(request_data)
        self.context7_request_pub.publish(request_msg)
        self.context7_request_count += 1
    
    def periodic_context7_query(self):
        """Periodically query Context7 for relevant documentation."""
        topics_to_query = [
            "ros2.agent_integration",
            "controller_best_practices", 
            "communication_patterns",
            "performance_optimization"
        ]
        
        for topic in topics_to_query:
            self.request_context7_documentation(topic)
    
    def bridge_operations(self):
        """Perform bridging operations with Context7-enhanced decision making."""
        # Check if we have relevant documentation in cache
        with self.context7_cache_lock:
            if 'ros2.agent_integration' in self.context7_cache:
                # Use cached documentation to make better bridging decisions
                doc_content = self.context7_cache['ros2.agent_integration']['content']
                
                # Apply Context7-recommended best practices
                self.apply_context7_best_practices(doc_content)
    
    def apply_context7_best_practices(self, documentation: str):
        """Apply Context7-recommended best practices to bridging operations."""
        # This would parse the documentation and apply recommended practices
        # For demonstration, we'll just log that we're applying them
        if "async" in documentation.lower():
            self.get_logger().info("Applying asynchronous communication patterns per Context7 recommendation")
        
        if "qos" in documentation.lower():
            self.get_logger().info("Optimizing QoS settings per Context7 recommendation")
    
    def process_agent_decision(self, decision: Dict[str, Any]):
        """Process agent decisions with Context7 guidance."""
        if 'action' in decision:
            action = decision['action']
            
            # Check Context7 for best practices for this action type
            action_doc_topic = f"action.{action}"
            if action_doc_topic in self.context7_cache:
                # Apply documented best practices for this action
                self.get_logger().info(f"Applying Context7 best practices for action: {action}")
            
            # Forward action to controller
            controller_cmd = String()
            controller_cmd.data = json.dumps(decision)
            self.controller_cmd_pub.publish(controller_cmd)
    
    def process_controller_feedback(self, feedback: Dict[str, Any]):
        """Process controller feedback with Context7 insights."""
        # Forward feedback to agent
        agent_response = String()
        agent_response.data = json.dumps(feedback)
        self.agent_cmd_pub.publish(agent_response)

def main(args=None):
    rclpy.init(args=args)
    node = Context7BridgingNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Context7 bridging node interrupted')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Context7 Integration for Documentation

### Dynamic Documentation Access

The integration of Context7 with Python agent-ROS controller bridges enables dynamic access to up-to-date documentation and best practices, which can significantly enhance both development and runtime capabilities:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import asyncio
import aiohttp
import json
import threading
import time
from typing import Dict, Any, Optional
from dataclasses import dataclass

@dataclass
class DocumentationRequest:
    """Request object for Context7 documentation."""
    topic: str
    priority: int = 1
    context: Dict[str, Any] = None

@dataclass
class DocumentationResponse:
    """Response object from Context7 documentation."""
    success: bool
    topic: str
    content: str
    metadata: Dict[str, Any]
    timestamp: float

class Context7DocumentationManager:
    """
    Manager for Context7 documentation access with caching and async operations.
    """
    
    def __init__(self, context7_endpoint: str = "http://localhost:8080"):
        self.context7_endpoint = context7_endpoint
        self.doc_cache: Dict[str, DocumentationResponse] = {}
        self.request_queue = asyncio.Queue()
        self.response_queue = asyncio.Queue()
        
        # Cache configuration
        self.cache_ttl = 300  # 5 minutes
        self.max_cache_size = 100
        
        # Async operations
        self.session = None
        self.running = False
        self.doc_lock = threading.Lock()
    
    async def initialize(self):
        """Initialize the documentation manager."""
        self.session = aiohttp.ClientSession()
        self.running = True
        
        # Start request processing
        asyncio.create_task(self.process_requests())
    
    async def cleanup(self):
        """Clean up resources."""
        self.running = False
        if self.session:
            await self.session.close()
    
    async def get_documentation(self, topic: str, use_cache: bool = True) -> Optional[DocumentationResponse]:
        """Get documentation for a specific topic."""
        # Check cache first
        if use_cache:
            with self.doc_lock:
                if topic in self.doc_cache:
                    cached_response = self.doc_cache[topic]
                    if time.time() - cached_response.timestamp < self.cache_ttl:
                        return cached_response
        
        # Request from Context7
        request = DocumentationRequest(topic=topic)
        await self.request_queue.put(request)
        
        # Wait for response
        timeout = 10.0  # 10 second timeout
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            try:
                response = self.response_queue.get_nowait()
                if response.topic == topic:
                    # Add to cache if successful
                    if response.success:
                        with self.doc_lock:
                            self.doc_cache[topic] = response
                            
                            # Limit cache size
                            if len(self.doc_cache) > self.max_cache_size:
                                # Remove oldest entries
                                oldest_key = min(
                                    self.doc_cache.keys(),
                                    key=lambda k: self.doc_cache[k].timestamp
                                )
                                del self.doc_cache[oldest_key]
                    
                    return response
            except asyncio.QueueEmpty:
                await asyncio.sleep(0.01)  # Sleep briefly before checking again
        
        return None
    
    async def process_requests(self):
        """Process documentation requests asynchronously."""
        while self.running:
            try:
                request = await asyncio.wait_for(self.request_queue.get(), timeout=1.0)
                
                # In a real implementation, this would make an MCP call to Context7
                # For demonstration, we'll use mock data
                response = await self._fetch_from_context7(request.topic)
                
                await self.response_queue.put(response)
                
            except asyncio.TimeoutError:
                continue  # Keep processing
            except Exception as e:
                # Create error response
                error_response = DocumentationResponse(
                    success=False,
                    topic=request.topic,
                    content="",
                    metadata={"error": str(e)},
                    timestamp=time.time()
                )
                await self.response_queue.put(error_response)
    
    async def _fetch_from_context7(self, topic: str) -> DocumentationResponse:
        """
        Fetch documentation from Context7 server.
        In a real implementation, this would connect to the Context7 MCP server.
        """
        # Mock implementation - in reality, this would make an MCP call
        await asyncio.sleep(0.1)  # Simulate network delay
        
        mock_docs = {
            "ros2.agent_integration": {
                "content": """
# ROS 2 Agent Integration Best Practices

## Communication Patterns
- Use asynchronous communication for non-critical data
- Implement proper error handling and retry mechanisms
- Consider message sizes and network bandwidth

## Performance Optimization
- Use appropriate QoS settings for your use case
- Implement efficient message serialization/deserialization
- Monitor resource usage and optimize accordingly

## Security Considerations
- Validate all incoming data from agents
- Implement access control for sensitive operations
- Use secure communication protocols when possible
                """,
                "metadata": {
                    "source": "ros2_docs",
                    "version": "humble",
                    "last_updated": "2024-12-14"
                }
            },
            "python_ai_ros_integration": {
                "content": """
# Python AI-ROS Integration Guidelines

## Architecture Patterns
- Implement clear separation between AI processing and ROS communication
- Use appropriate threading models for performance
- Consider real-time requirements for time-critical operations

## Data Management
- Efficient conversion between Python data structures and ROS messages
- Implement proper memory management for large datasets
- Use generators and iterators for memory-efficient processing

## Performance Optimization
- Minimize data copying between AI and ROS components
- Use appropriate message buffers and queue sizes
- Monitor and optimize inference performance
                """,
                "metadata": {
                    "source": "ai_ros_docs",
                    "version": "humble",
                    "last_updated": "2024-12-14"
                }
            }
        }
        
        doc_data = mock_docs.get(topic)
        if doc_data:
            return DocumentationResponse(
                success=True,
                topic=topic,
                content=doc_data["content"],
                metadata=doc_data["metadata"],
                timestamp=time.time()
            )
        else:
            return DocumentationResponse(
                success=False,
                topic=topic,
                content="",
                metadata={"error": f"No documentation found for {topic}"},
                timestamp=time.time()
            )

class Context7EnhancedAgentNode(Node):
    """
    ROS Node with Context7-enhanced agent integration.
    """
    
    def __init__(self):
        super().__init__('context7_enhanced_agent_node')
        
        # Initialize Context7 documentation manager
        self.doc_manager = Context7DocumentationManager()
        
        # Initialize async event loop
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        
        # Run the documentation manager initialization in the event loop
        self.loop.run_until_complete(self.doc_manager.initialize())
        
        # ROS communication
        self.qos_profile = 10
        
        # Publishers and subscribers
        self.agent_status_pub = self.create_publisher(String, 'agent_status', self.qos_profile)
        self.documentation_request_pub = self.create_publisher(String, 'documentation_requests', self.qos_profile)
        
        # Timer for periodic documentation checks
        self.doc_check_timer = self.create_timer(10.0, self.check_documentation_needs)
        
        self.get_logger().info('Context7 Enhanced Agent Node initialized')
    
    def check_documentation_needs(self):
        """Check if we need to request updated documentation."""
        # Example: check if we need documentation for current operations
        topics_to_check = [
            "ros2.agent_integration",
            "python_ai_ros_integration"
        ]
        
        for topic in topics_to_check:
            # Use threading to call async method from sync ROS timer
            future = asyncio.run_coroutine_threadsafe(
                self.doc_manager.get_documentation(topic), 
                self.loop
            )
            
            try:
                response = future.result(timeout=5.0)  # 5 second timeout
                if response and response.success:
                    self.get_logger().info(f'Documentation retrieved for: {topic}')
                    self.get_logger().debug(f'Content length: {len(response.content)} chars')
                else:
                    self.get_logger().warn(f'Failed to get documentation for: {topic}')
            except asyncio.TimeoutError:
                self.get_logger().error(f'Timeout getting documentation for: {topic}')
            except Exception as e:
                self.get_logger().error(f'Error getting documentation for {topic}: {e}')
    
    async def cleanup(self):
        """Clean up async resources."""
        await self.doc_manager.cleanup()

def main(args=None):
    rclpy.init(args=args)
    node = Context7EnhancedAgentNode()
    
    try:
        # Run ROS spin in a separate thread so we can handle async operations
        spin_thread = threading.Thread(target=lambda: rclpy.spin(node))
        spin_thread.start()
        
        # Wait for shutdown
        spin_thread.join()
        
    except KeyboardInterrupt:
        node.get_logger().info('Context7 enhanced agent interrupted')
    finally:
        # Clean up async resources
        future = asyncio.run_coroutine_threadsafe(node.cleanup(), node.loop)
        future.result(timeout=5.0)
        node.loop.close()
        
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Advanced Integration Patterns

### Hierarchical Control Architecture

Modern robotics systems often implement hierarchical control architectures where different levels of intelligence operate at different time scales and abstraction levels:

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String, Float64MultiArray
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import JointState, LaserScan
import time
import threading
from queue import Queue, Empty
import json
import numpy as np
from typing import Dict, Any, List

class HierarchicalControlNode(Node):
    """
    Hierarchical control architecture with multiple levels of intelligence:
    - High-level planning (seconds to minutes)
    - Mid-level coordination (100s of milliseconds to seconds)  
    - Low-level control (10s of milliseconds)
    """
    
    def __init__(self):
        super().__init__('hierarchical_control_node')
        
        # Define control hierarchy
        self.control_levels = {
            'high_level': {
                'name': 'planning',
                'frequency': 1.0,  # 1 Hz
                'responsibilities': ['mission planning', 'path planning', 'goal setting']
            },
            'mid_level': {
                'name': 'coordination', 
                'frequency': 10.0,  # 10 Hz
                'responsibilities': ['behavior coordination', 'task scheduling', 'resource allocation']
            },
            'low_level': {
                'name': 'control',
                'frequency': 100.0,  # 100 Hz
                'responsibilities': ['motor control', 'sensor fusion', 'safety monitoring']
            }
        }
        
        # Initialize state for each level
        self.level_states = {
            'high_level': {
                'current_goal': None,
                'mission_plan': [],
                'active': True
            },
            'mid_level': {
                'current_task': None,
                'subtasks': [],
                'active': True
            },
            'low_level': {
                'motor_commands': {},
                'safety_status': 'normal',
                'active': True
            }
        }
        
        # Communication interfaces for each level
        self.initialize_communication_interfaces()
        
        # Performance metrics
        self.metrics = {
            'high_level_cycles': 0,
            'mid_level_cycles': 0, 
            'low_level_cycles': 0,
            'high_level_time': [],
            'mid_level_time': [],
            'low_level_time': [],
            'message_throughput': 0
        }
        
        # Priority queues for cross-level communication
        self.high_to_mid_queue = Queue()
        self.mid_to_low_queue = Queue()
        self.low_to_mid_queue = Queue()
        self.mid_to_high_queue = Queue()
        
        # Start control loops
        self.start_control_loops()
        
        self.get_logger().info('Hierarchical Control Node initialized')
    
    def initialize_communication_interfaces(self):
        """Initialize ROS communication interfaces for each control level."""
        qos_profile = QoSProfile(depth=10)
        
        # High-level interfaces
        self.high_level_cmd_pub = self.create_publisher(
            String, 'high_level/commands', qos_profile
        )
        self.high_level_status_pub = self.create_publisher(
            String, 'high_level/status', qos_profile
        )
        self.high_level_sub = self.create_subscription(
            String, 'high_level/goals', self.high_level_callback, qos_profile
        )
        
        # Mid-level interfaces
        self.mid_level_cmd_pub = self.create_publisher(
            String, 'mid_level/commands', qos_profile
        )
        self.mid_level_status_pub = self.create_publisher(
            String, 'mid_level/status', qos_profile
        )
        self.mid_level_sub = self.create_subscription(
            String, 'mid_level/commands', self.mid_level_callback, qos_profile
        )
        
        # Low-level interfaces
        self.low_level_cmd_pub = self.create_publisher(
            Twist, 'cmd_vel', qos_profile  # For mobile base
        )
        self.low_level_joint_cmd_pub = self.create_publisher(
            JointState, 'joint_commands', qos_profile
        )
        self.low_level_status_pub = self.create_publisher(
            String, 'low_level/status', qos_profile
        )
        
        # Sensor interfaces (used by low/mid level)
        self.laser_sub = self.create_subscription(
            LaserScan, 'scan', self.laser_callback, qos_profile
        )
        self.joint_state_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, qos_profile
        )
        
        self.current_scan = None
        self.current_joints = None
    
    def laser_callback(self, msg):
        """Handle laser scan data (used by low/mid level)."""
        self.current_scan = msg
    
    def joint_state_callback(self, msg):
        """Handle joint state data (used by low/mid level)."""
        self.current_joints = msg
    
    def start_control_loops(self):
        """Start control loops for each hierarchy level."""
        # High-level planning loop (1 Hz)
        self.high_level_timer = self.create_timer(
            1.0 / self.control_levels['high_level']['frequency'],
            self.high_level_loop
        )
        
        # Mid-level coordination loop (10 Hz)  
        self.mid_level_timer = self.create_timer(
            1.0 / self.control_levels['mid_level']['frequency'], 
            self.mid_level_loop
        )
        
        # Low-level control loop (100 Hz)
        self.low_level_timer = self.create_timer(
            1.0 / self.control_levels['low_level']['frequency'],
            self.low_level_loop
        )
    
    def high_level_loop(self):
        """High-level planning loop."""
        start_time = time.time()
        
        # Process commands from higher authority or external goals
        self.process_high_level_input()
        
        # Execute high-level planning logic
        self.execute_high_level_planning()
        
        # Check for messages from lower levels
        self.process_mid_to_high_comm()
        
        # Update metrics
        cycle_time = time.time() - start_time
        self.metrics['high_level_time'].append(cycle_time)
        self.metrics['high_level_cycles'] += 1
        
        # Publish status
        self.publish_high_level_status()
    
    def mid_level_loop(self):
        """Mid-level coordination loop."""
        start_time = time.time()
        
        # Process high-level commands
        self.process_mid_level_input()
        
        # Execute coordination logic
        self.execute_mid_level_coordination()
        
        # Check cross-level communication
        self.process_high_to_mid_comm()
        self.process_low_to_mid_comm()
        
        # Update metrics
        cycle_time = time.time() - start_time
        self.metrics['mid_level_time'].append(cycle_time)
        self.metrics['mid_level_cycles'] += 1
        
        # Publish status
        self.publish_mid_level_status()
    
    def low_level_loop(self):
        """Low-level control loop."""
        start_time = time.time()
        
        # Process mid-level commands
        self.process_low_level_input()
        
        # Execute control algorithms (time-critical)
        self.execute_low_level_control()
        
        # Check for mid-level commands
        self.process_mid_to_low_comm()
        
        # Update metrics
        cycle_time = time.time() - start_time
        self.metrics['low_level_time'].append(cycle_time)
        self.metrics['low_level_cycles'] += 1
        
        # Publish status
        self.publish_low_level_status()
    
    def process_high_level_input(self):
        """Process high-level goals and commands."""
        # In a real system, this would handle mission planning
        # For demonstration, we'll set a simple goal periodically
        if self.metrics['high_level_cycles'] % 30 == 0:  # Every 30 seconds
            new_goal = {
                'type': 'navigation',
                'target': [5.0, 5.0, 0.0],
                'priority': 1
            }
            self.level_states['high_level']['current_goal'] = new_goal
    
    def execute_high_level_planning(self):
        """Execute high-level planning logic."""
        goal = self.level_states['high_level']['current_goal']
        if goal:
            # Generate subtasks for mid-level execution
            subtasks = self.generate_subtasks(goal)
            self.level_states['high_level']['mission_plan'] = subtasks
    
    def generate_subtasks(self, goal: Dict) -> List[Dict]:
        """Generate subtasks for a high-level goal."""
        if goal['type'] == 'navigation':
            # Simplified path planning - in reality would use A*, RRT, etc.
            current_pos = [0.0, 0.0, 0.0]  # Would come from localization
            target = goal['target']
            
            # Create path waypoints
            path_waypoints = self.create_path(current_pos[:2], target[:2])
            
            subtasks = []
            for i, waypoint in enumerate(path_waypoints):
                subtask = {
                    'id': f'nav_{i}',
                    'type': 'navigate_to_pose',
                    'target': [waypoint[0], waypoint[1], 0.0],
                    'priority': goal['priority']
                }
                subtasks.append(subtask)
            
            return subtasks
        
        return []
    
    def create_path(self, start: List[float], target: List[float], num_waypoints: int = 10) -> List[List[float]]:
        """Create a simple path between start and target."""
        start_np = np.array(start)
        target_np = np.array(target)
        
        path = []
        for i in range(num_waypoints + 1):
            alpha = i / num_waypoints
            waypoint = start_np + alpha * (target_np - start_np)
            path.append(waypoint.tolist())
        
        return path
    
    def process_mid_level_input(self):
        """Process mid-level tasks."""
        # Process current goal from high level
        if self.level_states['high_level']['mission_plan']:
            if not self.level_states['mid_level']['current_task']:
                # Get next task from plan
                if self.level_states['high_level']['mission_plan']:
                    self.level_states['mid_level']['current_task'] = \
                        self.level_states['high_level']['mission_plan'].pop(0)
    
    def execute_mid_level_coordination(self):
        """Execute mid-level coordination logic."""
        current_task = self.level_states['mid_level']['current_task']
        if current_task:
            # Check if task is complete
            task_complete = self.check_task_completion(current_task)
            
            if task_complete:
                self.level_states['mid_level']['current_task'] = None
                # Send completion message to high level
                completion_msg = String()
                completion_msg.data = json.dumps({
                    'type': 'task_complete',
                    'task_id': current_task['id'],
                    'status': 'success'
                })
                self.mid_to_high_queue.put(completion_msg)
    
    def check_task_completion(self, task: Dict) -> bool:
        """Check if a task is complete."""
        if task['type'] == 'navigate_to_pose':
            # Simplified completion check - in reality would use localization
            current_pos = [0.0, 0.0, 0.0]  # Would come from localization
            target = task['target']
            
            distance = np.linalg.norm(np.array(current_pos[:2]) - np.array(target[:2]))
            return distance < 0.2  # Consider complete within 0.2m
        
        return True
    
    def process_low_level_input(self):
        """Process low-level control commands."""
        current_task = self.level_states['mid_level']['current_task']
        if current_task and current_task['type'] == 'navigate_to_pose':
            # Generate velocity commands to reach target
            self.generate_navigation_commands(current_task['target'])
    
    def generate_navigation_commands(self, target: List[float]):
        """Generate low-level navigation commands."""
        # Simplified navigation - in reality would use proper path following
        current_pos = [0.0, 0.0, 0.0]  # Would come from localization
        
        direction = np.array(target[:2]) - np.array(current_pos[:2])
        distance = np.linalg.norm(direction)
        
        if distance > 0.1:  # If not close to target
            # Simple proportional control
            vel_linear = min(0.3, distance * 0.5)  # Max 0.3 m/s
            vel_angular = 0.0  # Simplified - would calculate heading error in reality
            
            cmd_msg = Twist()
            cmd_msg.linear.x = vel_linear
            cmd_msg.angular.z = vel_angular
            
            self.low_level_cmd_pub.publish(cmd_msg)
        else:
            # Stop when close to target
            stop_msg = Twist()
            self.low_level_cmd_pub.publish(stop_msg)
    
    def execute_low_level_control(self):
        """Execute time-critical low-level control."""
        # Safety checks
        if self.current_scan:
            # Check for obstacles
            obstacle_detected = self.check_for_obstacles()
            if obstacle_detected:
                self.level_states['low_level']['safety_status'] = 'obstacle_detected'
                # Emergency stop
                stop_msg = Twist()
                self.low_level_cmd_pub.publish(stop_msg)
            else:
                self.level_states['low_level']['safety_status'] = 'normal'
    
    def check_for_obstacles(self) -> bool:
        """Check laser scan for obstacles."""
        if self.current_scan:
            ranges = [r for r in self.current_scan.ranges if 0 < r < 1.0]
            return bool(ranges)  # Obstacle within 1m
        
        return False
    
    # Communication methods between levels
    def process_high_to_mid_comm(self):
        """Process messages from high level to mid level."""
        while True:
            try:
                if not self.high_to_mid_queue.empty():
                    msg = self.high_to_mid_queue.get_nowait()
                    self.handle_high_to_mid_message(msg)
            except Empty:
                break
    
    def process_mid_to_low_comm(self):
        """Process messages from mid level to low level."""
        while True:
            try:
                if not self.mid_to_low_queue.empty():
                    msg = self.mid_to_low_queue.get_nowait()
                    self.handle_mid_to_low_message(msg)
            except Empty:
                break
    
    def process_low_to_mid_comm(self):
        """Process messages from low level to mid level."""
        while True:
            try:
                if not self.low_to_mid_queue.empty():
                    msg = self.low_to_mid_queue.get_nowait()
                    self.handle_low_to_mid_message(msg)
            except Empty:
                break
    
    def process_mid_to_high_comm(self):
        """Process messages from mid level to high level."""
        while True:
            try:
                if not self.mid_to_high_queue.empty():
                    msg = self.mid_to_high_queue.get_nowait()
                    self.handle_mid_to_high_message(msg)
            except Empty:
                break
    
    def handle_high_to_mid_message(self, msg: String):
        """Handle message from high to mid level."""
        # Implementation placeholder
        pass
    
    def handle_mid_to_low_message(self, msg: String):
        """Handle message from mid to low level."""
        # Implementation placeholder
        pass
    
    def handle_low_to_mid_message(self, msg: String):
        """Handle message from low to mid level."""
        # Implementation placeholder
        pass
    
    def handle_mid_to_high_message(self, msg: String):
        """Handle message from mid to high level."""
        # Implementation placeholder
        pass
    
    def high_level_callback(self, msg):
        """Handle high-level commands from external sources."""
        try:
            command_data = json.loads(msg.data)
            self.level_states['high_level']['current_goal'] = command_data
        except json.JSONDecodeError:
            self.get_logger().error(f'Invalid high-level command: {msg.data}')
    
    def mid_level_callback(self, msg):
        """Handle mid-level commands."""
        try:
            command_data = json.loads(msg.data)
            # Process mid-level command
            pass
        except json.JSONDecodeError:
            self.get_logger().error(f'Invalid mid-level command: {msg.data}')
    
    def publish_high_level_status(self):
        """Publish high-level status."""
        status_msg = String()
        status_msg.data = json.dumps({
            'level': 'high',
            'active_goal': self.level_states['high_level']['current_goal'],
            'plan_progress': len(self.level_states['high_level']['mission_plan']),
            'timestamp': time.time()
        })
        self.high_level_status_pub.publish(status_msg)
    
    def publish_mid_level_status(self):
        """Publish mid-level status."""
        status_msg = String()
        status_msg.data = json.dumps({
            'level': 'mid',
            'current_task': self.level_states['mid_level']['current_task'],
            'subtasks_remaining': len(self.level_states['mid_level']['subtasks']),
            'timestamp': time.time()
        })
        self.mid_level_status_pub.publish(status_msg)
    
    def publish_low_level_status(self):
        """Publish low-level status."""
        status_msg = String()
        status_msg.data = json.dumps({
            'level': 'low',
            'safety_status': self.level_states['low_level']['safety_status'],
            'active': True,
            'timestamp': time.time()
        })
        self.low_level_status_pub.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    node = HierarchicalControlNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Hierarchical control node interrupted')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Real-World Examples

### Industrial Robot Integration

In industrial applications, Python agents are often used for high-level task planning and optimization, while ROS controllers handle precise motion control:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
from builtin_interfaces.msg import Duration
import time
import json
from typing import Dict, Any, List
import numpy as np

class IndustrialRobotIntegrationNode(Node):
    """
    Industrial robot integration with Python agent for task planning
    and ROS controllers for precise motion control.
    """
    
    def __init__(self):
        super().__init__('industrial_robot_integration_node')
        
        # Agent state for industrial applications
        self.agent_state = {
            'current_task': None,
            'task_queue': [],
            'robot_status': 'idle',
            'production_metrics': {
                'parts_completed': 0,
                'cycle_time_avg': 0.0,
                'quality_score': 100.0
            },
            'safety_zone': {
                'enabled': True,
                'protected_zones': []
            }
        }
        
        # Controller state
        self.controller_state = {
            'joint_positions': {},
            'desired_trajectory': None,
            'trajectory_progress': 0.0,
            'safety_limits': {
                'max_velocity': 1.0,
                'max_acceleration': 2.0,
                'collision_threshold': 0.1
            }
        }
        
        # ROS communication setup
        qos_profile = 10
        
        # Publishers for different controller interfaces
        self.joint_trajectory_pub = self.create_publisher(
            JointTrajectory, 'joint_trajectory', qos_profile
        )
        self.task_status_pub = self.create_publisher(
            String, 'task_status', qos_profile
        )
        
        # Subscribers for feedback
        self.joint_state_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, qos_profile
        )
        self.controller_state_sub = self.create_subscription(
            JointTrajectoryControllerState, 'controller_state', self.controller_state_callback, qos_profile
        )
        
        # Industrial task interface
        self.task_command_sub = self.create_subscription(
            String, 'task_commands', self.task_command_callback, qos_profile
        )
        
        # Safety system interface
        self.safety_system_sub = self.create_subscription(
            String, 'safety_system', self.safety_callback, qos_profile
        )
        
        # Control loop timers
        self.agent_timer = self.create_timer(0.5, self.agent_control_loop)  # 2 Hz for task planning
        self.controller_timer = self.create_timer(0.01, self.controller_loop)  # 100 Hz for motion control
        
        self.get_logger().info('Industrial Robot Integration Node initialized')
    
    def joint_state_callback(self, msg):
        """Handle joint state feedback."""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.controller_state['joint_positions'][name] = msg.position[i]
    
    def controller_state_callback(self, msg):
        """Handle controller state feedback."""
        # Update trajectory progress and other controller metrics
        pass
    
    def task_command_callback(self, msg):
        """Handle task commands from industrial system."""
        try:
            task_data = json.loads(msg.data)
            self.agent_state['task_queue'].append(task_data)
        except json.JSONDecodeError:
            self.get_logger().error(f'Invalid task command: {msg.data}')
    
    def safety_callback(self, msg):
        """Handle safety system events."""
        try:
            safety_data = json.loads(msg.data)
            if safety_data.get('emergency_stop'):
                self.emergency_stop()
            elif safety_data.get('safety_reset'):
                self.safety_reset()
        except json.JSONDecodeError:
            self.get_logger().error(f'Invalid safety message: {msg.data}')
    
    def agent_control_loop(self):
        """High-level industrial task planning and management."""
        # Check for new tasks
        if self.agent_state['task_queue'] and not self.agent_state['current_task']:
            self.agent_state['current_task'] = self.agent_state['task_queue'].pop(0)
        
        if self.agent_state['current_task']:
            # Execute task planning
            self.execute_task_planning()
        
        # Update production metrics
        self.update_production_metrics()
        
        # Publish task status
        self.publish_task_status()
    
    def execute_task_planning(self):
        """Plan and execute industrial tasks."""
        task = self.agent_state['current_task']
        
        if task['type'] == 'assembly':
            # Generate assembly trajectory
            trajectory = self.generate_assembly_trajectory(task)
            self.execute_trajectory(trajectory)
        
        elif task['type'] == 'inspection':
            # Generate inspection trajectory
            trajectory = self.generate_inspection_trajectory(task)
            self.execute_trajectory(trajectory)
        
        elif task['type'] == 'material_handling':
            # Generate material handling trajectory
            trajectory = self.generate_material_handling_trajectory(task)
            self.execute_trajectory(trajectory)
    
    def generate_assembly_trajectory(self, task: Dict) -> JointTrajectory:
        """Generate trajectory for assembly task."""
        trajectory = JointTrajectory()
        trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        
        # Generate approach, assembly, and retreat movements
        waypoints = [
            # Pre-position
            [0.0, -1.0, 0.0, -1.5, 0.0, 0.0],
            # Approach position
            [0.1, -1.1, 0.1, -1.4, 0.1, 0.0],
            # Assembly position  
            [0.15, -1.15, 0.15, -1.35, 0.15, 0.0],
            # Post-assembly
            [0.1, -1.1, 0.1, -1.4, 0.1, 0.0]
        ]
        
        # Create trajectory points with timing
        for i, waypoint in enumerate(waypoints):
            point = JointTrajectoryPoint()
            point.positions = waypoint
            point.velocities = [0.0] * len(waypoint)  # Zero velocity at waypoints
            point.time_from_start = Duration(sec=i+1, nanosec=0)  # 1 second per point
            trajectory.points.append(point)
        
        return trajectory
    
    def generate_inspection_trajectory(self, task: Dict) -> JointTrajectory:
        """Generate trajectory for inspection task."""
        # For inspection, move through predefined inspection points
        trajectory = JointTrajectory()
        trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        
        # Define inspection waypoints
        inspection_points = task.get('inspection_points', [
            [0.5, -0.5, 0.5, -1.0, 0.0, 0.0],
            [0.6, -0.6, 0.4, -1.1, 0.1, 0.0],
            [0.55, -0.55, 0.45, -1.05, 0.05, 0.0]
        ])
        
        for i, point_pos in enumerate(inspection_points):
            point = JointTrajectoryPoint()
            point.positions = point_pos
            point.velocities = [0.0] * len(point_pos)
            point.time_from_start = Duration(sec=i+1, nanosec=0)
            trajectory.points.append(point)
        
        return trajectory
    
    def generate_material_handling_trajectory(self, task: Dict) -> JointTrajectory:
        """Generate trajectory for material handling task."""
        trajectory = JointTrajectory()
        trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        
        # Define pickup and drop-off positions
        pickup_pos = task.get('pickup_position', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        dropoff_pos = task.get('dropoff_position', [1.0, 1.0, 0.0, 0.0, 0.0, 0.0])
        
        # Create safe approach, pickup, transport, and dropoff trajectory
        waypoints = [
            # Pre-pickup approach
            [pickup_pos[0], pickup_pos[1], pickup_pos[2] + 0.2, pickup_pos[3], pickup_pos[4], pickup_pos[5]],  # Safe height
            # Pickup position
            pickup_pos,
            # Post-pickup (with load)
            [pickup_pos[0], pickup_pos[1], pickup_pos[2] + 0.2, pickup_pos[3], pickup_pos[4], pickup_pos[5]],
            # Transport to dropoff
            [dropoff_pos[0], dropoff_pos[1], dropoff_pos[2] + 0.2, dropoff_pos[3], dropoff_pos[4], dropoff_pos[5]],
            # Dropoff position
            dropoff_pos,
            # Post-dropoff
            [dropoff_pos[0], dropoff_pos[1], dropoff_pos[2] + 0.2, dropoff_pos[3], dropoff_pos[4], dropoff_pos[5]]
        ]
        
        for i, waypoint in enumerate(waypoints):
            point = JointTrajectoryPoint()
            point.positions = waypoint
            point.velocities = [0.0] * len(waypoint)
            point.time_from_start = Duration(sec=i+1, nanosec=0)
            trajectory.points.append(point)
        
        return trajectory
    
    def execute_trajectory(self, trajectory: JointTrajectory):
        """Execute a joint trajectory."""
        # Add trajectory ID for tracking
        trajectory.header.stamp = self.get_clock().now().to_msg()
        
        # Publish trajectory to controller
        self.joint_trajectory_pub.publish(trajectory)
        
        # Update controller state
        self.controller_state['desired_trajectory'] = trajectory
        self.controller_state['trajectory_progress'] = 0.0
    
    def controller_loop(self):
        """Low-level motion control loop."""
        # Check trajectory execution status
        if self.controller_state['desired_trajectory']:
            self.monitor_trajectory_execution()
        
        # Safety monitoring
        self.safety_monitoring()
    
    def monitor_trajectory_execution(self):
        """Monitor trajectory execution progress."""
        # In a real system, this would monitor actual vs desired joint positions
        # For simulation, we'll just advance progress
        self.controller_state['trajectory_progress'] += 0.01  # 1% per cycle at 100Hz
        
        if self.controller_state['trajectory_progress'] >= 1.0:
            # Trajectory completed
            self.trajectory_completed()
    
    def trajectory_completed(self):
        """Handle trajectory completion."""
        self.get_logger().info('Trajectory completed')
        
        # Mark current task as complete
        if self.agent_state['current_task']:
            self.agent_state['production_metrics']['parts_completed'] += 1
            self.agent_state['current_task'] = None  # Move to next task on next cycle
        
        # Reset trajectory
        self.controller_state['desired_trajectory'] = None
        self.controller_state['trajectory_progress'] = 0.0
    
    def safety_monitoring(self):
        """Monitor safety systems and limits."""
        # Check joint limits and safety zones
        for joint_name, position in self.controller_state['joint_positions'].items():
            # Check if joint is within safe limits (simplified)
            if abs(position) > 3.0:  # Example limit
                self.get_logger().warn(f'Joint {joint_name} exceeds safety limit: {position}')
                self.emergency_stop()
                return
    
    def emergency_stop(self):
        """Execute emergency stop."""
        self.get_logger().fatal('EMERGENCY STOP ACTIVATED')
        
        # Stop all motion
        stop_trajectory = JointTrajectory()
        stop_trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        
        stop_point = JointTrajectoryPoint()
        stop_point.positions = [0.0] * 6  # Return to safe position
        stop_point.velocities = [0.0] * 6
        stop_point.time_from_start = Duration(sec=0, nanosec=100000000)  # 0.1 second
        
        stop_trajectory.points = [stop_point]
        self.joint_trajectory_pub.publish(stop_trajectory)
        
        # Update states
        self.agent_state['robot_status'] = 'emergency_stopped'
        self.controller_state['desired_trajectory'] = None
    
    def safety_reset(self):
        """Reset from emergency stop."""
        self.get_logger().info('Safety reset')
        self.agent_state['robot_status'] = 'idle'
    
    def update_production_metrics(self):
        """Update production metrics."""
        # Calculate average cycle time if we have completed tasks
        if self.agent_state['production_metrics']['parts_completed'] > 0:
            # In a real system, this would track actual cycle times
            self.agent_state['production_metrics']['cycle_time_avg'] = 25.0  # Simplified
    
    def publish_task_status(self):
        """Publish current task and system status."""
        status_msg = String()
        status_msg.data = json.dumps({
            'current_task': self.agent_state['current_task'],
            'robot_status': self.agent_state['robot_status'],
            'production_metrics': self.agent_state['production_metrics'],
            'timestamp': time.time()
        })
        self.task_status_pub.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    node = IndustrialRobotIntegrationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Industrial robot integration interrupted')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Service Robot Integration

For service robots, Python agents often handle customer interaction, navigation planning, and multi-modal perception:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from builtin_interfaces.msg import Duration
import time
import json
import numpy as np
from typing import Dict, Any, List
import asyncio

class ServiceRobotIntegrationNode(Node):
    """
    Service robot integration with Python agent for customer interaction
    and multi-modal perception with ROS controllers for navigation and manipulation.
    """
    
    def __init__(self):
        super().__init__('service_robot_integration_node')
        
        # Agent state for service applications
        self.agent_state = {
            'current_interaction': None,
            'navigation_goals': [],
            'customer_queue': [],
            'service_status': 'idle',
            'perception_data': {
                'faces': [],
                'objects': [],
                'person_count': 0
            },
            'navigation_map': None,
            'service_metrics': {
                'interactions_completed': 0,
                'navigation_success_rate': 0.0,
                'customer_satisfaction': 0.0
            }
        }
        
        # Navigation controller state
        self.nav_controller_state = {
            'current_pose': np.array([0.0, 0.0, 0.0]),  # x, y, theta
            'current_goal': None,
            'path': [],
            'navigation_active': False,
            'obstacles': []
        }
        
        # Communication setup
        qos_profile = 10
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos_profile)
        self.goal_pub = self.create_publisher(PoseStamped, 'move_base_simple/goal', qos_profile)
        self.interaction_status_pub = self.create_publisher(String, 'interaction_status', qos_profile)
        
        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, qos_profile)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, qos_profile)
        self.camera_sub = self.create_subscription(Image, 'camera/image_raw', self.camera_callback, qos_profile)
        
        # Service interface
        self.customer_call_sub = self.create_subscription(
            String, 'customer_call', self.customer_call_callback, qos_profile
        )
        self.service_request_sub = self.create_subscription(
            String, 'service_requests', self.service_request_callback, qos_profile
        )
        
        # Control loops
        self.interaction_timer = self.create_timer(0.5, self.interaction_control_loop)  # Customer interaction
        self.navigation_timer = self.create_timer(0.1, self.navigation_control_loop)   # Navigation
        self.perception_timer = self.create_timer(0.2, self.perception_processing_loop) # Perception
        self.monitoring_timer = self.create_timer(1.0, self.system_monitoring_loop)    # System monitoring
        
        self.get_logger().info('Service Robot Integration Node initialized')
    
    def odom_callback(self, msg):
        """Handle odometry for localization."""
        self.nav_controller_state['current_pose'] = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            2 * np.arctan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)  # Convert quaternion to yaw
        ])
    
    def scan_callback(self, msg):
        """Handle laser scan for obstacle detection."""
        ranges = [r for r in msg.ranges if 0 < r < 5.0]
        if ranges:
            self.nav_controller_state['obstacles'] = ranges
    
    def camera_callback(self, msg):
        """Handle camera image for perception."""
        # In a real system, this would perform computer vision processing
        # For simulation, we'll mock some perception results
        self.agent_state['perception_data']['person_count'] = 2  # Mock detection
        self.agent_state['perception_data']['faces'] = [
            {'x': 0.5, 'y': 0.3, 'confidence': 0.9},
            {'x': 0.7, 'y': 0.4, 'confidence': 0.85}
        ]
    
    def customer_call_callback(self, msg):
        """Handle customer calls/requests."""
        try:
            call_data = json.loads(msg.data)
            customer_request = {
                'id': call_data.get('customer_id', 'unknown'),
                'request': call_data.get('request', ''),
                'location': call_data.get('location', [0.0, 0.0]),
                'priority': call_data.get('priority', 1),
                'timestamp': time.time()
            }
            self.agent_state['customer_queue'].append(customer_request)
        except json.JSONDecodeError:
            self.get_logger().error(f'Invalid customer call: {msg.data}')
    
    def service_request_callback(self, msg):
        """Handle service requests from other systems."""
        try:
            request_data = json.loads(msg.data)
            # Process service request based on type
            if request_data.get('type') == 'navigation':
                self.add_navigation_goal(request_data['goal'])
            elif request_data.get('type') == 'interaction':
                self.start_customer_interaction(request_data)
        except json.JSONDecodeError:
            self.get_logger().error(f'Invalid service request: {msg.data}')
    
    def interaction_control_loop(self):
        """Handle customer interactions and service management."""
        # Check for new customer calls
        if self.agent_state['customer_queue']:
            # Sort by priority and time
            self.agent_state['customer_queue'].sort(key=lambda x: (-x['priority'], x['timestamp']))
            next_customer = self.agent_state['customer_queue'].pop(0)
            
            # Start interaction with customer
            self.start_customer_interaction(next_customer)
        
        # Continue ongoing interactions
        if self.agent_state['current_interaction']:
            self.continue_interaction()
        
        # Update service status
        self.publish_interaction_status()
    
    def start_customer_interaction(self, customer_request: Dict):
        """Start a new customer interaction."""
        self.agent_state['current_interaction'] = {
            'customer_id': customer_request['id'],
            'request': customer_request['request'],
            'start_time': time.time(),
            'phase': 'approach',
            'conversation_history': []
        }
        
        self.agent_state['service_status'] = 'interacting'
        
        # Navigate to customer if needed
        customer_location = customer_request.get('location')
        if customer_location:
            self.navigate_to_customer(customer_location)
    
    def continue_interaction(self):
        """Continue the current customer interaction."""
        interaction = self.agent_state['current_interaction']
        
        if interaction['phase'] == 'approach':
            # Check if we've reached the customer
            if self.nav_controller_state['navigation_active'] == False:
                interaction['phase'] = 'engagement'
                interaction['conversation_history'].append({
                    'speaker': 'robot',
                    'message': 'Hello! How can I assist you today?',
                    'timestamp': time.time()
                })
        
        elif interaction['phase'] == 'engagement':
            # Handle customer interaction - in reality would use NLP/Speech systems
            if self.agent_state['perception_data']['person_count'] > 0:
                # Acknowledge customer presence
                interaction['conversation_history'].append({
                    'speaker': 'robot',
                    'message': 'I can see you. How can I help?',
                    'timestamp': time.time()
                })
        
        elif interaction['phase'] == 'fulfillment':
            # Fulfill customer request
            pass
        
        elif interaction['phase'] == 'conclusion':
            # Conclude interaction
            self.complete_interaction()
    
    def complete_interaction(self):
        """Complete current customer interaction."""
        interaction = self.agent_state['current_interaction']
        
        if interaction:
            interaction_time = time.time() - interaction['start_time']
            
            # Update metrics
            self.agent_state['service_metrics']['interactions_completed'] += 1
            self.agent_state['service_metrics']['customer_satisfaction'] = 4.5  # Mock rating
            
            self.get_logger().info(f'Interaction completed with customer {interaction["customer_id"]} in {interaction_time:.1f}s')
            
            # Reset interaction state
            self.agent_state['current_interaction'] = None
            
            # Return to idle status
            self.agent_state['service_status'] = 'idle'
    
    def navigate_to_customer(self, location: List[float]):
        """Navigate to customer location."""
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = 'map'
        goal_msg.pose.position.x = location[0]
        goal_msg.pose.position.y = location[1]
        goal_msg.pose.position.z = 0.0
        goal_msg.pose.orientation.w = 1.0  # No rotation
        
        self.goal_pub.publish(goal_msg)
        
        self.nav_controller_state['current_goal'] = location
        self.nav_controller_state['navigation_active'] = True
    
    def navigation_control_loop(self):
        """Handle navigation and path planning."""
        # Check if navigation is active and if we've reached the goal
        if self.nav_controller_state['navigation_active'] and self.nav_controller_state['current_goal']:
            current_pos = self.nav_controller_state['current_pose']
            goal = self.nav_controller_state['current_goal']
            
            # Calculate distance to goal
            distance = np.linalg.norm(current_pos[:2] - np.array(goal[:2]))
            
            if distance < 0.5:  # Within 0.5m of goal
                self.nav_controller_state['navigation_active'] = False
                self.nav_controller_state['current_goal'] = None
                
                # If this was for customer interaction, update interaction phase
                if self.agent_state['current_interaction'] and self.agent_state['current_interaction']['phase'] == 'approach':
                    self.agent_state['current_interaction']['phase'] = 'engagement'
    
    def perception_processing_loop(self):
        """Process perception data for service applications."""
        # Update customer detection count
        if self.agent_state['perception_data']['person_count'] > 0:
            # Adjust behavior based on perceived environment
            pass
    
    def system_monitoring_loop(self):
        """Monitor overall system status."""
        # Update navigation success rate
        if self.agent_state['service_metrics']['interactions_completed'] > 0:
            self.agent_state['service_metrics']['navigation_success_rate'] = 0.95  # Mock value
        
        # Log system status periodically
        self.get_logger().debug(f'Service status: {self.agent_state["service_status"]}, '
                               f'Customers in queue: {len(self.agent_state["customer_queue"])}')
    
    def add_navigation_goal(self, goal: Dict[str, Any]):
        """Add a navigation goal to the system."""
        location = [goal['x'], goal['y'], goal.get('theta', 0.0)]
        self.navigate_to_customer(location)
    
    def publish_interaction_status(self):
        """Publish interaction and service status."""
        status_msg = String()
        status_msg.data = json.dumps({
            'current_interaction': self.agent_state['current_interaction'],
            'service_status': self.agent_state['service_status'],
            'customer_queue_size': len(self.agent_state['customer_queue']),
            'perception_data': self.agent_state['perception_data'],
            'navigation_active': self.nav_controller_state['navigation_active'],
            'metrics': self.agent_state['service_metrics'],
            'timestamp': time.time()
        })
        self.interaction_status_pub.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ServiceRobotIntegrationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Service robot integration interrupted')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Best Practices

### Performance Optimization

Optimizing the performance of Python agent-ROS controller bridges requires careful attention to several key areas:

1. **Threading and Concurrency Management**: Separate time-critical control loops from computationally intensive AI processing
2. **Memory Management**: Use object pooling and efficient data structures to minimize garbage collection impact
3. **Communication Optimization**: Use appropriate QoS settings and message buffering strategies
4. **Real-time Considerations**: Implement proper real-time scheduling where needed

### Security Considerations

Security is paramount in robotic systems, especially when bridging different execution environments:

1. **Input Validation**: Always validate data received from AI agents before processing
2. **Access Control**: Implement proper authorization for sensitive control commands
3. **Data Encryption**: Use encrypted communication channels for sensitive data
4. **Sandboxing**: Consider running AI components in sandboxed environments

### Design Patterns

The following design patterns are particularly useful for Python agent-ROS controller integration:

1. **Observer Pattern**: For handling sensor data updates
2. **Command Pattern**: For encapsulating control commands
3. **State Pattern**: For managing different operational states
4. **Strategy Pattern**: For selecting different control algorithms

## Performance Optimization

### Efficient Message Handling

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
from collections import deque
import threading

class OptimizedAgentBridgeNode(Node):
    """
    Optimized bridge with efficient message handling and performance monitoring.
    """
    
    def __init__(self):
        super().__init__('optimized_agent_bridge_node')
        
        # Optimized message handling using deques for efficient appends/pops
        self.incoming_messages = deque(maxlen=1000)
        self.outgoing_messages = deque(maxlen=1000)
        
        # Message processing pool
        self.processing_pool = deque(maxlen=10)
        
        # Performance metrics
        self.metrics = {
            'messages_processed': 0,
            'avg_processing_time': deque(maxlen=100),
            'message_queue_depth': deque(maxlen=100),
            'processing_threads': 0
        }
        
        qos_profile = 10
        
        # Publishers and subscribers
        self.agent_pub = self.create_publisher(String, 'optimized_commands', qos_profile)
        self.agent_sub = self.create_subscription(
            String, 'optimized_responses', self.optimized_message_callback, qos_profile
        )
        
        # Optimized processing timer
        self.process_timer = self.create_timer(0.005, self.optimized_process_loop)  # 200 Hz
        
        # Performance monitoring timer
        self.metrics_timer = self.create_timer(1.0, self.log_performance_metrics)
        
        self.get_logger().info('Optimized Agent Bridge Node initialized')
        
        # Start processing threads for heavy computation
        self.start_processing_threads()
    
    def optimized_message_callback(self, msg):
        """Optimized message callback to minimize processing time."""
        # Add to processing queue immediately
        self.incoming_messages.append({
            'data': msg.data,
            'timestamp': time.time(),
            'source': 'agent'
        })
        
        # Update metrics
        self.metrics['message_queue_depth'].append(len(self.incoming_messages))
    
    def optimized_process_loop(self):
        """Optimized processing loop with minimal overhead."""
        start_time = time.time()
        
        # Process a limited number of messages per cycle to maintain timing
        processed_count = 0
        max_per_cycle = 10  # Limit to prevent blocking
        
        while self.incoming_messages and processed_count < max_per_cycle:
            try:
                msg = self.incoming_messages.popleft()
                self.process_optimized_message(msg)
                processed_count += 1
            except IndexError:
                break  # Queue is empty
        
        # Calculate and store processing time
        processing_time = time.time() - start_time
        self.metrics['avg_processing_time'].append(processing_time)
        self.metrics['messages_processed'] += processed_count
    
    def process_optimized_message(self, msg):
        """Process message with optimized algorithms."""
        # In a real implementation, this would contain optimized processing
        # For demonstration, we'll just acknowledge the message
        response = {
            'original_message': msg['data'],
            'processed_at': msg['timestamp'],
            'status': 'processed'
        }
        
        response_msg = String()
        response_msg.data = str(response)
        self.agent_pub.publish(response_msg)
    
    def start_processing_threads(self):
        """Start background processing threads for heavy computation."""
        def processing_worker():
            while rclpy.ok():
                # Process heavy computations in background
                time.sleep(0.01)  # Yield to other threads
        
        # Start worker threads (be conservative with thread count)
        for i in range(1):  # Typically 1 processing thread is sufficient
            thread = threading.Thread(target=processing_worker, daemon=True)
            thread.start()
    
    def log_performance_metrics(self):
        """Log performance metrics periodically."""
        if self.metrics['avg_processing_time']:
            avg_time = sum(self.metrics['avg_processing_time']) / len(self.metrics['avg_processing_time'])
            self.get_logger().info(
                f'Performance - Processed: {self.metrics["messages_processed"]}, '
                f'Avg time: {avg_time*1000:.2f}ms, '
                f'Queue depth: {len(self.incoming_messages)}'
            )

def main(args=None):
    rclpy.init(args=args)
    node = OptimizedAgentBridgeNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Optimized bridge interrupted')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Security Considerations

### Secure Communication Implementation

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import hashlib
import hmac
import time
import json
from typing import Dict, Any

class SecureAgentControllerNode(Node):
    """
    Secure agent-controller bridge with authentication and validation.
    """
    
    def __init__(self):
        super().__init__('secure_agent_controller_node')
        
        # Security configuration
        self.security_config = {
            'shared_secret': 'your_shared_secret_here',  # Should be loaded from secure storage
            'message_timeout': 30.0,  # 30 seconds
            'max_message_size': 1024 * 10,  # 10KB maximum
            'allowed_publishers': ['agent_node_1', 'agent_node_2']  # Whitelist
        }
        
        # Security state
        self.security_state = {
            'message_counter': 0,
            'last_message_time': time.time(),
            'security_violations': 0
        }
        
        qos_profile = 10
        
        # Secure publishers and subscribers
        self.secure_command_pub = self.create_publisher(String, 'secure_commands', qos_profile)
        self.secure_command_sub = self.create_subscription(
            String, 'secure_agent_commands', self.secure_command_callback, qos_profile
        )
        
        # Status publisher
        self.security_status_pub = self.create_publisher(String, 'security_status', qos_profile)
        
        # Security monitoring timer
        self.security_timer = self.create_timer(5.0, self.security_monitoring)
        
        self.get_logger().info('Secure Agent-Controller Node initialized')
    
    def secure_command_callback(self, msg):
        """Handle incoming commands with security validation."""
        try:
            # Parse the message which should contain security metadata
            message_data = json.loads(msg.data)
            
            # Validate message format
            if not self.validate_message_format(message_data):
                self.log_security_violation('Invalid message format')
                return
            
            # Verify message integrity
            if not self.verify_message_integrity(message_data):
                self.log_security_violation('Message integrity check failed')
                return
            
            # Check message freshness (prevent replay attacks)
            if not self.check_message_freshness(message_data):
                self.log_security_violation('Message too old (replay attack?)')
                return
            
            # Validate message size
            if len(msg.data) > self.security_config['max_message_size']:
                self.log_security_violation('Message exceeds maximum size')
                return
            
            # Process the validated command
            self.process_validated_command(message_data)
            
        except json.JSONDecodeError:
            self.log_security_violation('Invalid JSON in message')
        except Exception as e:
            self.log_security_violation(f'Error processing secure message: {e}')
    
    def validate_message_format(self, message_data: Dict[str, Any]) -> bool:
        """Validate the format of the incoming message."""
        required_fields = ['content', 'signature', 'timestamp', 'sender_id']
        for field in required_fields:
            if field not in message_data:
                return False
        
        # Additional validation can be added here
        return True
    
    def verify_message_integrity(self, message_data: Dict[str, Any]) -> bool:
        """Verify the integrity of the message using HMAC."""
        try:
            # Extract message components
            content = message_data['content']
            timestamp = message_data['timestamp']
            received_signature = message_data['signature']
            
            # Create the message to verify
            message_to_verify = f"{content}{timestamp}{message_data.get('sender_id', '')}"
            
            # Calculate expected signature
            expected_signature = hmac.new(
                self.security_config['shared_secret'].encode(),
                message_to_verify.encode(),
                hashlib.sha256
            ).hexdigest()
            
            # Compare signatures
            return hmac.compare_digest(expected_signature, received_signature)
        
        except Exception:
            return False
    
    def check_message_freshness(self, message_data: Dict[str, Any]) -> bool:
        """Check if the message is fresh (not too old)."""
        try:
            message_time = float(message_data['timestamp'])
            current_time = time.time()
            
            # Check if message is within acceptable time window
            return (current_time - message_time) <= self.security_config['message_timeout']
        except (ValueError, KeyError):
            return False
    
    def process_validated_command(self, message_data: Dict[str, Any]):
        """Process a command that has passed all security checks."""
        content = message_data['content']
        sender_id = message_data['sender_id']
        
        # Log the successful processing
        self.get_logger().info(f'Processing secure command from {sender_id}')
        
        # In a real implementation, process the command content here
        # For demonstration, we'll just acknowledge it
        ack_message = {
            'acknowledged': True,
            'original_sender': sender_id,
            'processed_at': time.time()
        }
        
        ack_msg = String()
        ack_msg.data = json.dumps(ack_message)
        self.secure_command_pub.publish(ack_msg)
    
    def log_security_violation(self, violation_description: str):
        """Log security violations."""
        self.security_state['security_violations'] += 1
        self.get_logger().error(f'SECURITY VIOLATION: {violation_description}')
        
        # Publish security status
        status_msg = String()
        status_msg.data = json.dumps({
            'security_violations': self.security_state['security_violations'],
            'violation_description': violation_description,
            'timestamp': time.time()
        })
        self.security_status_pub.publish(status_msg)
    
    def security_monitoring(self):
        """Monitor security metrics."""
        # Could implement additional security checks here
        # such as rate limiting, anomaly detection, etc.
        pass
    
    def generate_secure_message(self, content: Any, sender_id: str) -> str:
        """Generate a secure message with signature."""
        message_data = {
            'content': content,
            'timestamp': time.time(),
            'sender_id': sender_id
        }
        
        # Create signature
        message_str = f"{content}{message_data['timestamp']}{sender_id}"
        signature = hmac.new(
            self.security_config['shared_secret'].encode(),
            message_str.encode(),
            hashlib.sha256
        ).hexdigest()
        
        message_data['signature'] = signature
        
        return json.dumps(message_data)

def main(args=None):
    rclpy.init(args=args)
    node = SecureAgentControllerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Secure agent-controller interrupted')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Future Developments

### Emerging Trends in Agent-Controller Integration

The field of Python agent-ROS controller integration continues to evolve with emerging trends:

1. **Edge AI Integration**: Running AI models directly on robot hardware
2. **5G and Low-Latency Networks**: Ultra-low latency communication for real-time control
3. **Digital Twin Technology**: Virtual replicas for enhanced planning and testing
4. **Federated Learning**: Distributed machine learning across robotic fleets
5. **Quantum Computing**: Potential future applications for optimization

### Advanced Integration Techniques

As robotics systems become more sophisticated, new integration techniques are emerging:

1. **Machine Learning Pipeline Integration**: Direct pipeline from sensor data to control
2. **AutoML for Controller Tuning**: Automated optimization of controller parameters
3. **Reinforcement Learning Integration**: Continuous learning and adaptation
4. **Multi-Agent Systems**: Coordination between multiple intelligent agents

## Summary

The bridging of Python agents to ROS controllers represents a crucial integration point in modern robotic systems. This comprehensive guide has explored the architecture, implementation patterns, and best practices for creating robust, efficient, and secure bridges between high-level intelligent agents and low-level real-time controllers.

The integration encompasses multiple architectural patterns from direct in-process integration to message-passing architectures with separate process isolation. Each approach offers different trade-offs in terms of performance, complexity, and maintainability, and the choice depends on the specific requirements of the robotic application.

Context7 integration adds significant value by providing dynamic access to up-to-date documentation and best practices, enabling more maintainable and well-documented systems. This integration can enhance both development-time tooling and runtime decision-making capabilities.

Performance optimization remains critical, particularly in time-sensitive applications where Python's garbage collection and dynamic nature can impact real-time execution. Techniques such as careful resource management, efficient data structures, and appropriate threading models help mitigate these challenges.

Security considerations are paramount in robotic systems, where unauthorized access could lead to safety risks or operational disruptions. Implementing secure communication protocols, input validation, and access control mechanisms is essential for production systems.

Real-world applications demonstrate the practical implementation of these concepts in diverse domains from industrial automation to service robotics. Each application domain presents unique challenges and requirements that must be carefully considered in the integration design.

The future of agent-controller integration will likely see advances in AI/ML integration, with increasingly sophisticated agents capable of real-time learning and adaptation. Edge computing, 5G connectivity, and emerging AI technologies will continue to shape the evolution of these systems.

Through careful application of the patterns and practices outlined in this guide, developers can create robust, performant, and maintainable bridges between Python agents and ROS controllers that enable sophisticated robotic capabilities while maintaining system reliability and safety.