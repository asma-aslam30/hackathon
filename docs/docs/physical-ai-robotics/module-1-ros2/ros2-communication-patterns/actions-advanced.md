---
sidebar_position: 2
sidebar_label: Understanding ROS 2 Actions - Deep Dive
---

# ROS 2 Actions - A Comprehensive Guide with Context7 Integration

## Table of Contents
1. [Introduction](#introduction)
2. [Deep Technical Analysis](#deep-technical-analysis)
3. [Understanding ROS 2 Actions](#understanding-ros-2-actions)
4. [Action Architecture and Design Patterns](#action-architecture-and-design-patterns)
5. [Implementation Best Practices](#implementation-best-practices)
6. [Advanced Action Patterns](#advanced-action-patterns)
7. [Context7 Integration for Documentation](#context7-integration-for-documentation)
8. [Real-World Examples](#real-world-examples)
9. [Performance Optimization](#performance-optimization)
10. [Security and Safety Considerations](#security-and-safety-considerations)
11. [Testing and Debugging](#testing-and-debugging)
12. [Future Developments](#future-developments)
13. [Summary](#summary)

## Introduction

ROS 2 Actions represent a sophisticated communication pattern that bridges the gap between services and topics in the Robot Operating System. They provide a mechanism for long-running operations that require feedback during execution, the ability to cancel operations, and detailed final results. Unlike services which provide synchronous request-response communication, or topics which offer asynchronous one-way communication, Actions are designed for complex operations that unfold over time and require ongoing interaction between the requesting client and the executing server.

The significance of Actions in modern robotic systems cannot be overstated. They enable the implementation of sophisticated behaviors such as robot navigation where operators need to monitor progress and potentially cancel routes if circumstances change, manipulation tasks that require real-time feedback about grasp success, and calibration procedures that provide continuous updates on progress and quality metrics. This communication paradigm has become essential as robotic systems grow in complexity and autonomy.

The evolution from ROS 1 to ROS 2 brought substantial improvements to the Action framework, addressing critical limitations while incorporating modern distributed system principles. The integration of Context7 documentation systems enhances the development process by providing immediate access to up-to-date best practices, API references, and implementation guidelines. This integration streamlines the development workflow and ensures that implementations align with current best practices.

This comprehensive guide explores the latest developments in ROS 2 Actions as of 2025, focusing on advanced implementation techniques, performance optimization strategies, and integration patterns with Context7 documentation systems. We examine how these communication patterns enable more sophisticated robotic behaviors while maintaining the reliability and safety required for real-world deployment.

## Deep Technical Analysis

### Evolution from ROS 1 to ROS 2 Actions

The Action architecture in ROS 2 represents a fundamental redesign from its predecessor in ROS 1, addressing critical limitations while incorporating modern distributed system principles. The original ROS 1 Action system relied on a centralized master architecture that could create bottlenecks and single points of failure. ROS 2 Actions, however, leverage the distributed architecture built on DDS middleware, providing native support for:

1. **Decentralized Operation**: No central master required for action communication
2. **Native Security**: Built-in security mechanisms at the middleware level
3. **Quality of Service (QoS) Policies**: Configurable behavior for different operational requirements
4. **Multi-platform Support**: Native support for various operating systems and architectures
5. **Real-time Performance**: Deterministic behavior suitable for safety-critical applications

### Core Action Architecture

ROS 2 Actions implement a three-part communication pattern that differs significantly from topics and services:

**Goals**: Initiate an action request with initial parameters, similar to service requests but designed for long-running operations.

**Feedback**: Continuous updates during action execution, providing real-time progress information without blocking the execution thread.

**Results**: Final outcome of the action execution, including completion status, success/failure indicators, and any computed results.

This tripartite structure provides a rich communication paradigm that enables sophisticated coordination between nodes, particularly in scenarios where operators need to monitor and potentially intervene in long-running processes.

### Action Implementation Architecture

The implementation of Actions in ROS 2 leverages the DDS middleware through a sophisticated pattern that combines multiple topic-based communications under a unified interface:

1. **Goal Topic**: For sending goal requests
2. **Cancel Topic**: For sending cancellation requests
3. **Status Topic**: For sharing action status among participants
4. **Feedback Topic**: For continuous feedback updates
5. **Result Topic**: For final result delivery

This multi-topic approach enables the complex state management required for Actions while maintaining the distributed nature of the ROS 2 architecture.

### State Machine Implementation

ROS 2 Actions operate through a well-defined state machine that provides visibility into the operation's status:

1. **PENDING**: Goal received but not yet processed
2. **ACTIVE**: Goal accepted and currently executing
3. **PREEMPTING**: Goal is being cancelled by request
4. **SUCCEEDED**: Goal completed successfully
5. **ABORTED**: Goal failed due to an error
6. **CANCELED**: Goal successfully cancelled
7. **RECALLING**: Goal recall requested before execution started
8. **REJECTED**: Goal rejected by the server
9. **LOST**: No contact with the action client/server

### Advanced Technical Considerations

Modern implementations of ROS 2 Actions incorporate several critical technical considerations:

**Real-time Performance**: Actions must be designed to work within real-time constraints, especially for safety-critical applications where deterministic behavior is essential.

**Memory Management**: Proper resource allocation and deallocation patterns are crucial for preventing memory leaks in long-running operations.

**Error Handling**: Comprehensive error handling strategies must be implemented to manage various failure modes gracefully.

**Security Integration**: Actions must incorporate security measures including authentication, authorization, and data encryption where appropriate.

**Performance Monitoring**: Built-in metrics collection and monitoring capabilities enable system optimization and debugging.

## Understanding ROS 2 Actions

### What Are Actions?

Actions in ROS 2 represent a sophisticated communication pattern specifically designed for long-running tasks that require feedback, the ability to cancel operations, and detailed result reporting. They combine the best aspects of services (request-response pattern) and topics (asynchronous communication) while adding unique capabilities for monitoring progress and managing long-duration operations.

Key Characteristics of Actions:
- **Long-Running Operations**: Designed for tasks that take seconds to minutes to complete
- **Feedback During Execution**: Continuous updates on progress and status
- **Cancellation Capability**: Ability to abort operations before completion
- **Detailed Results**: Rich result structure including success/failure indicators
- **State Management**: Sophisticated state handling for complex operations

### When to Use Actions vs Services vs Topics

Understanding when to use Actions versus other communication patterns is crucial for effective system design:

**Use Actions for:**
- Navigation to waypoints with progress monitoring
- Robot calibration procedures with intermediate status
- Complex manipulation tasks with feedback
- Planning algorithms that take time to compute
- Any operation that needs to be cancellable
- Tasks requiring detailed progress reporting

**Use Services for:**
- Quick configuration changes
- Triggering immediate actions
- Querying system status
- Operations that should complete quickly
- Synchronous request-response operations

**Use Topics for:**
- Streaming sensor data
- Broadcasting robot state
- Continuous control commands
- Asynchronous one-way communication
- High-frequency data transmission

### Action Message Structure

The technical implementation of Actions involves a specialized message structure that extends the standard ROS 2 message format. Each action interface defines three distinct message types:

1. **Action Goal**: Contains parameters for initiating the action
2. **Action Feedback**: Contains ongoing progress information
3. **Action Result**: Contains the final outcome of the action

These message types are compiled into a single action interface definition that creates specialized publishers, subscribers, and clients that understand the action lifecycle.

```action
# Define goal, result, and feedback messages for an action

# Goal definition
int32 target_position_x
int32 target_position_y
float64 tolerance

---
# Result definition
bool success
string message
int32 final_position_x
int32 final_position_y
duration processing_time

---
# Feedback definition
int32 current_position_x
int32 current_position_y
float64 distance_remaining
float64 progress_percentage
string status_message
```

### Quality of Service (QoS) Considerations

ROS 2 Actions incorporate Quality of Service policies that enable behavior customization:

1. **Reliability**: Ensuring goal and result delivery
2. **Durability**: Whether late-joining nodes receive action information
3. **History**: How many action requests/results to maintain
4. **Deadline**: Maximum time for action completion
5. **Liveliness**: Ensuring action server availability

## Action Architecture and Design Patterns

### Basic Action Server Implementation

```python
import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.node import Node
from rclpy.qos import QoSProfile
import time

class FibonacciActionServer(Node):
    """
    Example action server implementing the Fibonacci sequence calculation
    as a long-running, cancellable operation with continuous feedback.
    """
    
    def __init__(self):
        super().__init__('fibonacci_action_server')
        
        # Create action server with appropriate QoS
        self._action_server = ActionServer(
            self,
            Fibonacci,  # Action interface (defined in .action file)
            'fibonacci',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )
        
        self.get_logger().info('Fibonacci action server initialized')

    def goal_callback(self, goal_request):
        """
        Accept or reject goal requests.
        
        Args:
            goal_request: The incoming goal request from the client
            
        Returns:
            GoalResponse indicating whether to accept or reject the goal
        """
        self.get_logger().info('Received goal request')
        
        # Validate the goal request
        if goal_request.order < 1:
            self.get_logger().warn('Goal order must be greater than 0')
            return GoalResponse.REJECT
        
        if goal_request.order > 100:  # Prevent excessive computation
            self.get_logger().warn('Goal order too large, rejecting')
            return GoalResponse.REJECT
        
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """
        Accept or reject goal cancellation requests.
        
        Args:
            goal_handle: Handle to the goal being cancelled
            
        Returns:
            CancelResponse indicating acceptance or rejection of cancellation
        """
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        """
        Execute the goal as an asynchronous coroutine.
        
        Args:
            goal_handle: Handle to the goal that enables feedback and status reporting
            
        Returns:
            Fibonacci.Result containing the outcome of the execution
        """
        self.get_logger().info('Executing goal...')

        # Initialize result message
        result = Fibonacci.Result()
        result.sequence = [0, 1]

        # Give feedback periodically as the sequence progresses
        feedback_msg = Fibonacci.Feedback()

        # Calculate Fibonacci sequence
        for i in range(1, goal_handle.request.order):
            # Check if the goal has been cancelled
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Goal was cancelled')
                result.sequence = result.sequence[:i+1]  # Return partial sequence
                goal_handle.canceled()
                return result

            # Calculate next Fibonacci number
            if i < len(result.sequence):
                # For the first few numbers, just update index
                continue
            else:
                next_fib = result.sequence[i-1] + result.sequence[i-2]
                result.sequence.append(next_fib)

            # Publish feedback
            feedback_msg.sequence = result.sequence
            goal_handle.publish_feedback(feedback_msg)
            
            # Simulate some processing time
            time.sleep(0.1)

            # Log progress periodically
            if i % 5 == 0:
                self.get_logger().info(f'Current sequence length: {len(result.sequence)}')

        # Check if goal was cancelled during execution
        if goal_handle.is_cancel_requested:
            self.get_logger().info('Goal was cancelled during execution')
            goal_handle.canceled()
            result.sequence = result.sequence[:-1]  # Remove last element that wasn't completed
            return result

        # Set goal status to succeeded and return result
        goal_handle.succeed()
        self.get_logger().info('Returning result: {0}'.format(result.sequence))
        return result

def main(args=None):
    rclpy.init(args=args)

    fibonacci_action_server = FibonacciActionServer()

    try:
        rclpy.spin(fibonacci_action_server)
    except KeyboardInterrupt:
        fibonacci_action_server.get_logger().info('Action server stopped cleanly')
    finally:
        fibonacci_action_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Action Client Implementation

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile

class FibonacciActionClient(Node):
    """
    Example action client that sends requests to the Fibonacci action server.
    """
    
    def __init__(self):
        super().__init__('fibonacci_action_client')
        
        # Create action client
        self._action_client = ActionClient(
            self,
            Fibonacci,  # Action interface
            'fibonacci'
        )

    def send_goal(self, order):
        """
        Send a goal to the action server.
        
        Args:
            order: The order of the Fibonacci sequence to calculate
        """
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        # Ensure action server is available
        self._action_client.wait_for_server()

        # Send goal asynchronously
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """
        Handle the goal response from the action server.
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal was rejected by server')
            return

        self.get_logger().info('Goal accepted by server, waiting for result')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """
        Handle the result from the action server.
        """
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.sequence))

    def feedback_callback(self, feedback_msg):
        """
        Handle feedback messages from the action server.
        """
        self.get_logger().info(
            'Received feedback: {0}'.format(feedback_msg.feedback.sequence)
        )

def main(args=None):
    rclpy.init(args=args)

    action_client = FibonacciActionClient()

    # Send a goal
    action_client.send_goal(10)

    try:
        rclpy.spin(action_client)
    except KeyboardInterrupt:
        action_client.get_logger().info('Action client stopped cleanly')
    finally:
        action_client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Advanced Action Patterns

#### Conditional Action Execution

```python
import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.node import Node
from rclpy.qos import QoSProfile
import asyncio
import threading
import time
from typing import Dict, Any, Optional

class ConditionalActionServer(Node):
    """
    Advanced action server with conditional execution logic.
    This implementation demonstrates how to handle complex action flows
    with conditional branching and state management.
    """
    
    def __init__(self):
        super().__init__('conditional_action_server')
        
        # Create action server with reentrant callback group for concurrency
        self._action_server = ActionServer(
            self,
            ComplexAction,  # Custom action interface
            'conditional_action',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=rclpy.callback_groups.ReentrantCallbackGroup()
        )
        
        # State management for complex operations
        self._active_goals = {}  # Store active goal information
        self._execution_state = {}  # Track execution state
        self._resource_manager = ResourceManager()  # Manage shared resources
        
        # Publishers for monitoring
        self._status_publisher = self.create_publisher(
            String, 'action_status', QoSProfile(depth=10)
        )
        
        self.get_logger().info('Conditional action server initialized with advanced features')

    def goal_callback(self, goal_request):
        """
        Smart goal evaluation with resource and condition checking.
        """
        self.get_logger().info(f'Received goal: {goal_request}')
        
        # Check resource availability
        required_resources = self._analyze_resource_requirements(goal_request)
        if not self._resource_manager.are_available(required_resources):
            self.get_logger().warn('Insufficient resources for goal, rejecting')
            return GoalResponse.REJECT
        
        # Check conditions based on current system state
        if not self._check_execution_conditions(goal_request):
            self.get_logger().warn('Execution conditions not met, rejecting')
            return GoalResponse.REJECT
        
        # Check for conflicts with ongoing goals
        if self._has_conflicting_goals(goal_request):
            self.get_logger().warn('Goal conflicts with ongoing operations, rejecting')
            return GoalResponse.REJECT
        
        self.get_logger().info('Goal accepted, sufficient resources and conditions met')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """
        Smart cancellation that considers the execution state.
        """
        goal_id = goal_handle.goal_id
        
        # Cancel only if the goal is in an appropriate state
        if self._can_cancel_goal(goal_id):
            self.get_logger().info(f'Cancelling goal: {goal_id}')
            return CancelResponse.ACCEPT
        else:
            self.get_logger().warn(f'Cannot cancel goal {goal_id} in current state')
            return CancelResponse.REJECT

    async def execute_callback(self, goal_handle):
        """
        Execute the goal with complex conditional logic and state management.
        """
        goal_id = goal_handle.goal_id
        self.get_logger().info(f'Executing goal: {goal_id}')
        
        # Initialize execution state
        execution_id = self._initialize_execution_state(goal_handle)
        
        try:
            # Acquire necessary resources
            required_resources = self._analyze_resource_requirements(goal_handle.request)
            await self._resource_manager.acquire_async(required_resources)
            
            # Execute the main action logic with conditional branching
            result = await self._execute_main_logic(goal_handle, execution_id)
            
        except Exception as e:
            self.get_logger().error(f'Execution failed: {str(e)}')
            result = self._create_error_result(str(e))
        finally:
            # Release resources
            await self._resource_manager.release_async(required_resources)
            
            # Clean up execution state
            self._cleanup_execution_state(execution_id)
        
        return result

    async def _execute_main_logic(self, goal_handle, execution_id):
        """
        Execute the main action logic with conditional branching.
        """
        result = ComplexAction.Result()
        feedback_msg = ComplexAction.Feedback()
        
        # Analyze the goal request to determine the execution path
        execution_path = self._determine_execution_path(goal_handle.request)
        
        for step in execution_path:
            # Check for cancellation at each major step
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Goal cancelled during execution')
                goal_handle.canceled()
                return result
            
            # Execute the current step
            step_result = await self._execute_step(step, goal_handle)
            
            if step_result.success:
                # Update feedback
                feedback_msg.current_step = step.name
                feedback_msg.progress = step_result.progress
                feedback_msg.status = step_result.status
                goal_handle.publish_feedback(feedback_msg)
                
                # Check for conditional branching based on step results
                conditional_branch = self._check_conditional_branch(step_result)
                if conditional_branch:
                    # Branch to a different execution path
                    next_steps = self._get_conditional_path(conditional_branch)
                    execution_path.extend(next_steps)
            else:
                # Step failed, determine if we can continue or should abort
                if step_result.is_critical:
                    self.get_logger().error(f'Critical step failed: {step.name}')
                    goal_handle.abort()
                    result.success = False
                    result.error_message = f'Critical failure in step: {step.name}'
                    return result
                else:
                    self.get_logger().warn(f'Step failed but not critical: {step.name}')
                    # Continue with execution
        
        # Check final conditions before indicating success
        if self._verify_final_conditions(goal_handle):
            goal_handle.succeed()
            result.success = True
            result.completion_time = time.time()
        else:
            goal_handle.abort()
            result.success = False
            result.error_message = 'Final conditions not met'
        
        return result

    def _determine_execution_path(self, request):
        """
        Determine the execution path based on the request.
        """
        # This would implement complex condition analysis to determine
        # the appropriate sequence of operations
        path = []
        
        if request.operation_type == 'navigation':
            path.extend([
                ExecutionStep('initialize_navigation', self._init_navigation),
                ExecutionStep('plan_path', self._plan_path),
                ExecutionStep('execute_path', self._execute_path),
                ExecutionStep('verify_arrival', self._verify_arrival)
            ])
        elif request.operation_type == 'manipulation':
            path.extend([
                ExecutionStep('locate_object', self._locate_object),
                ExecutionStep('plan_grasp', self._plan_grasp),
                ExecutionStep('execute_grasp', self._execute_grasp),
                ExecutionStep('verify_grasp', self._verify_grasp)
            ])
        elif request.operation_type == 'calibration':
            path.extend([
                ExecutionStep('initialize_calibration', self._init_calibration),
                ExecutionStep('collect_data', self._collect_data),
                ExecutionStep('process_calibration', self._process_calibration),
                ExecutionStep('validate_calibration', self._validate_calibration)
            ])
        
        # Add conditional steps based on request parameters
        if request.requires_verification:
            path.append(ExecutionStep('post_operation_verification', self._post_verify))
        
        return path

    async def _execute_step(self, step, goal_handle):
        """
        Execute a single step in the execution path.
        """
        try:
            start_time = time.time()
            
            # Execute the step's function
            step_result = await step.function(goal_handle)
            
            # Calculate performance metrics
            execution_time = time.time() - start_time
            
            result = StepExecutionResult(
                success=step_result.get('success', True),
                progress=step_result.get('progress', 0.0),
                status=step_result.get('status', 'completed'),
                is_critical=step_result.get('critical', False),
                execution_time=execution_time
            )
            
            return result
            
        except Exception as e:
            self.get_logger().error(f'Step {step.name} failed: {str(e)}')
            
            return StepExecutionResult(
                success=False,
                progress=0.0,
                status=f'error: {str(e)}',
                is_critical=True,
                execution_time=0.0
            )

    def _check_conditional_branch(self, step_result):
        """
        Check if a conditional branch should be taken based on step result.
        """
        # Analyze step result to determine if we should branch
        if step_result.status.startswith('branch_'):
            return step_result.status.split('_', 1)[1]  # Extract branch name
        
        return None

    def _get_conditional_path(self, branch_name):
        """
        Get the execution path for a conditional branch.
        """
        conditional_paths = {
            'recovery': [
                ExecutionStep('attempt_recovery', self._attempt_recovery),
                ExecutionStep('retry_operation', self._retry_operation)
            ],
            'verification': [
                ExecutionStep('additional_verification', self._additional_verify),
                ExecutionStep('confirm_results', self._confirm_results)
            ],
            'optimization': [
                ExecutionStep('optimize_parameters', self._optimize_params),
                ExecutionStep('re_execute_optimized', self._re_execute_optimized)
            ]
        }
        
        return conditional_paths.get(branch_name, [])

    async def _init_navigation(self, goal_handle):
        """Initialize navigation operation."""
        self.get_logger().info('Initializing navigation')
        await asyncio.sleep(1.0)  # Simulate initialization
        return {'success': True, 'progress': 0.1, 'status': 'initialized'}

    async def _plan_path(self, goal_handle):
        """Plan navigation path."""
        self.get_logger().info('Planning navigation path')
        await asyncio.sleep(2.0)  # Simulate path planning
        return {'success': True, 'progress': 0.3, 'status': 'path_planned'}

    async def _execute_path(self, goal_handle):
        """Execute navigation path."""
        self.get_logger().info('Executing navigation path')
        # Simulate path execution with periodic updates
        for i in range(5):
            if goal_handle.is_cancel_requested:
                return {'success': False, 'progress': i/5, 'status': 'cancelled'}
            await asyncio.sleep(0.5)
        
        return {'success': True, 'progress': 0.9, 'status': 'path_executed'}

    async def _verify_arrival(self, goal_handle):
        """Verify arrival at destination."""
        self.get_logger().info('Verifying arrival at destination')
        await asyncio.sleep(1.0)  # Simulate verification
        return {'success': True, 'progress': 1.0, 'status': 'arrival_verified'}

from dataclasses import dataclass
from typing import Callable, Any

@dataclass
class ExecutionStep:
    """Represents a single step in an execution path."""
    name: str
    function: Callable  # The function to execute this step
    required_resources: list = None  # Resources needed for this step
    timeout: float = 30.0  # Timeout for this step
    critical: bool = True  # Whether this step is critical for success

@dataclass
class StepExecutionResult:
    """Result of executing a single step."""
    success: bool
    progress: float
    status: str
    is_critical: bool
    execution_time: float

class ResourceManager:
    """
    Resource manager for coordinating access to shared resources.
    """
    
    def __init__(self):
        self._available_resources = {
            'cpu': 100,  # Percentage
            'memory': 8192,  # MB
            'gpu': 100,  # Percentage
            'network_bandwidth': 1000,  # Mbps
            'motors': 6,  # Number of motors available
            'sensors': 10  # Number of sensors available
        }
        
        self._reserved_resources = {}  # Track reserved resources per goal
        self._resource_locks = {}  # Locks for resource coordination
        
    async def acquire_async(self, resources_needed: Dict[str, Any]):
        """Acquire required resources asynchronously."""
        # In a real implementation, this would handle resource allocation
        # For this example, we'll simulate the process
        for resource, amount in resources_needed.items():
            if resource in self._available_resources:
                if self._available_resources[resource] >= amount:
                    self._available_resources[resource] -= amount
                else:
                    raise Exception(f"Insufficient {resource}: need {amount}, available {self._available_resources[resource]}")
    
    async def release_async(self, resources_used: Dict[str, Any]):
        """Release previously acquired resources asynchronously."""
        for resource, amount in resources_used.items():
            if resource in self._available_resources:
                self._available_resources[resource] += amount

def main_conditional_action(args=None):
    """Main function for conditional action server."""
    rclpy.init(args=args)
    
    conditional_action_server = ConditionalActionServer()
    
    try:
        rclpy.spin(conditional_action_server)
    except KeyboardInterrupt:
        conditional_action_server.get_logger().info('Conditional action server stopped cleanly')
    finally:
        conditional_action_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main_conditional_action()
```

#### Asynchronous Action Execution with Threading

```python
import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.callback_groups import ReentrantCallbackGroup
import asyncio
import threading
from concurrent.futures import ThreadPoolExecutor, as_completed
import time
from typing import Dict, Any, Optional

class AsyncActionServer(Node):
    """
    Action server with asynchronous execution using threading and async/await.
    This implementation demonstrates how to handle long-running operations
    that might block without interfering with the ROS main loop.
    """
    
    def __init__(self):
        super().__init__('async_action_server')
        
        # Create a thread pool executor for long-running operations
        self._executor = ThreadPoolExecutor(max_workers=4)
        
        # Use reentrant callback group for thread safety
        callback_group = ReentrantCallbackGroup()
        
        # Create action server
        self._action_server = ActionServer(
            self,
            LongRunningAction,  # Custom action interface
            'async_long_running_action',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=callback_group
        )
        
        # Track active futures for cancellation
        self._active_futures = {}
        
        self.get_logger().info('Asynchronous action server initialized')

    def goal_callback(self, goal_request):
        """Accept all goals for this example."""
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Handle cancellation requests."""
        goal_id = goal_handle.goal_id
        self.get_logger().info(f'Received cancel request for goal: {goal_id}')
        
        # Check if we can cancel the ongoing operation
        future = self._active_futures.get(goal_id, None)
        if future and not future.done():
            # In Python, we can't truly cancel a running thread
            # Instead, we'll rely on the operation checking for cancellation
            goal_handle.set_cancellation_flag()
            return CancelResponse.ACCEPT
        
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        """
        Execute the goal asynchronously using threading.
        """
        goal_id = goal_handle.goal_id
        self.get_logger().info(f'Executing goal asynchronously: {goal_id}')
        
        # Submit the long-running operation to the thread pool
        future = self._executor.submit(
            self._execute_long_running_operation,
            goal_handle,
            self.get_clock().now().nanoseconds
        )
        
        # Store the future for potential cancellation
        self._active_futures[goal_id] = future
        
        try:
            # Wait for the operation to complete
            # We'll check for cancellation periodically
            result = self._wait_for_completion_with_cancellation(
                future, goal_handle, timeout=300.0  # 5 minute timeout
            )
        except Exception as e:
            self.get_logger().error(f'Execution failed: {str(e)}')
            result = LongRunningAction.Result()
            result.success = False
            result.error_message = str(e)
        finally:
            # Clean up the future reference
            self._active_futures.pop(goal_id, None)
        
        return result

    def _wait_for_completion_with_cancellation(self, future, goal_handle, timeout=300.0):
        """
        Wait for completion while checking for cancellation.
        """
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            # Check if goal is cancelled
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Goal cancelled during execution')
                goal_handle.canceled()
                result = LongRunningAction.Result()
                result.success = False
                result.error_message = "Goal cancelled by client"
                return result
            
            # Check if thread has completed
            if future.done():
                try:
                    return future.result()
                except Exception as e:
                    result = LongRunningAction.Result()
                    result.success = False
                    result.error_message = f"Thread execution error: {str(e)}"
                    return result
            
            # Sleep briefly to yield to other operations
            time.sleep(0.1)
        
        # Timeout occurred
        self.get_logger().error('Execution timed out')
        result = LongRunningAction.Result()
        result.success = False
        result.error_message = "Execution timed out"
        return result

    def _execute_long_running_operation(self, goal_handle, start_time_ns):
        """
        Execute the actual long-running operation in a separate thread.
        This function should not use ROS-specific APIs as it runs in a different thread.
        """
        # Get the request data
        request = goal_handle.request
        
        # Initialize result
        result = LongRunningAction.Result()
        result.success = True
        result.steps_completed = 0
        result.total_steps = request.expected_steps if hasattr(request, 'expected_steps') else 10
        
        # Simulate a long-running operation with progress updates
        for step in range(result.total_steps):
            # Check for cancellation at each step
            if goal_handle._cancellation_flag:  # This would be a custom flag
                result.success = False
                result.error_message = "Cancelled by user"
                break
            
            # Simulate work
            time.sleep(0.5)  # Simulate 0.5 seconds of work per step
            
            # Update steps completed
            result.steps_completed += 1
            
            # Calculate progress percentage
            progress = (result.steps_completed / result.total_steps) * 100.0
            
            # Create feedback message
            feedback = LongRunningAction.Feedback()
            feedback.current_step = step + 1
            feedback.total_steps = result.total_steps
            feedback.progress_percentage = progress
            feedback.status_message = f"Completed step {step + 1} of {result.total_steps}"
            feedback.elapsed_time = (time.time_ns() - start_time_ns) / 1e9
            
            # Publish feedback (this won't work directly from another thread,
            # in a real implementation, you'd need to use the executor to publish)
            # For this example, we'll note that feedback would be published
            
            # Log progress periodically
            if step % 2 == 0:  # Log every 2 steps
                print(f"Progress: {progress:.1f}% - {feedback.status_message}")
        
        # Operation completed (successfully or due to cancellation)
        result.completion_time = (time.time_ns() - start_time_ns) / 1e9
        
        return result

class AsyncActionClient(Node):
    """
    Asynchronous action client that demonstrates non-blocking communication.
    """
    
    def __init__(self):
        super().__init__('async_action_client')
        
        # Create action client
        self._action_client = ActionClient(
            self,
            LongRunningAction,
            'async_long_running_action'
        )
        
        # Timer to periodically send goals
        self._send_goal_timer = self.create_timer(5.0, self.send_goal_callback)
        
        self._goal_count = 0

    def send_goal_callback(self):
        """Send goals periodically."""
        self._goal_count += 1
        
        goal_msg = LongRunningAction.Goal()
        goal_msg.expected_steps = 10
        goal_msg.priority = 5
        goal_msg.parameters = {"operation": f"long_running_op_{self._goal_count}"}
        
        self.get_logger().info(f'Sending goal {self._goal_count}')
        
        # Send goal asynchronously
        future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        
        # Add callback for goal response
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle goal response."""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        
        self.get_logger().info('Goal accepted, getting result')
        
        # Get result asynchronously
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        """Handle result."""
        result = future.result().result
        
        if result.success:
            self.get_logger().info(f'Goal succeeded. Completed {result.steps_completed}/{result.total_steps} steps in {result.completion_time:.2f}s')
        else:
            self.get_logger().info(f'Goal failed: {result.error_message}')

    def feedback_callback(self, feedback_msg):
        """Handle feedback."""
        self.get_logger().info(
            f'Feedback: {feedback_msg.current_step}/{feedback_msg.total_steps} - '
            f'{feedback_msg.progress_percentage:.1f}% - {feedback_msg.status_message}'
        )

def main_async_action(args=None):
    """Main function for async action demonstration."""
    rclpy.init(args=args)
    
    # Create both server and client nodes
    server = AsyncActionServer()
    client = AsyncActionClient()
    
    # Create a multi-threaded executor to handle both nodes
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(server)
    executor.add_node(client)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        server.get_logger().info('Async action server stopped cleanly')
        client.get_logger().info('Async action client stopped cleanly')
    finally:
        executor.shutdown()
        server.destroy_node()
        client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main_async_action()
```

### Context7 Integration for Documentation

The integration of Context7 documentation systems enhances the development of ROS 2 Actions by providing immediate access to up-to-date best practices, API references, and implementation guidelines. This integration streamlines the development workflow and ensures that implementations align with current best practices.

```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from std_msgs.msg import String
import asyncio
import aiohttp
import json
from typing import Dict, Any, Optional
import time

class Context7ActionIntegrationNode(Node):
    """
    Node that demonstrates Context7 integration for ROS 2 Actions documentation.
    This shows how to dynamically access documentation and best practices for action implementations.
    """
    
    def __init__(self):
        super().__init__('context7_action_integration_node')
        
        # Publishers for documentation requests and responses
        self._doc_request_pub = self.create_publisher(
            String, 'context7_action_documentation_requests', 10
        )
        self._doc_response_sub = self.create_subscription(
            String, 'context7_action_documentation_responses', 
            self.doc_response_callback, 10
        )
        
        # Action server with documentation integration
        self._action_server = ActionServer(
            self,
            DocumentedAction,
            'context7_integrated_action',
            execute_callback=self.documented_execute_callback,
            goal_callback=self.documented_goal_callback,
            cancel_callback=self.documented_cancel_callback
        )
        
        # Documentation cache
        self._doc_cache = {}
        self._cache_ttl = 300  # 5 minutes
        
        # Performance metrics for documentation access
        self._doc_metrics = {
            'requests': 0,
            'responses': 0,
            'cache_hits': 0,
            'cache_misses': 0,
            'avg_response_time': []
        }
        
        # Timer for periodic documentation updates
        self._doc_update_timer = self.create_timer(30.0, self.periodic_doc_update)
        
        self.get_logger().info('Context7 Action Integration Node initialized')

    def doc_response_callback(self, msg):
        """Handle responses from Context7 documentation system."""
        try:
            response_data = json.loads(msg.data)
            topic = response_data.get('topic')
            content = response_data.get('content')
            
            if topic and content:
                self._doc_cache[topic] = {
                    'content': content,
                    'timestamp': time.time(),
                    'source': response_data.get('source', 'unknown')
                }
                
                self._doc_metrics['responses'] += 1
                self.get_logger().info(f'Documentation cached for: {topic[:50]}...')
            else:
                self.get_logger().warn('Invalid documentation response format')
                
        except json.JSONDecodeError:
            self.get_logger().error(f'Invalid JSON in documentation response: {msg.data}')
        except Exception as e:
            self.get_logger().error(f'Error processing documentation response: {e}')

    def documented_goal_callback(self, goal_request):
        """
        Goal callback with Context7 documentation integration.
        """
        self.get_logger().info('Received goal request - checking documentation')
        
        # Check for relevant documentation
        action_type = goal_request.type if hasattr(goal_request, 'type') else 'generic'
        doc_topic = f"ros2.action.{action_type}.best_practices"
        
        # Check cache first
        doc_content = self._get_documentation(doc_topic)
        if doc_content:
            self.get_logger().info(f'Best practices for {action_type} retrieved from documentation')
            
            # Apply best practices based on documentation
            if "validation" in doc_content.get('content', '').lower():
                if not self._validate_request_with_docs(goal_request, doc_content):
                    self.get_logger().warn('Request validation failed based on documentation')
                    return GoalResponse.REJECT
        
        # Check for rate limiting based on documentation
        if not self._check_rate_limit_by_docs(action_type):
            self.get_logger().warn(f'Rate limit exceeded for {action_type}')
            return GoalResponse.REJECT
        
        return GoalResponse.ACCEPT

    def documented_cancel_callback(self, goal_handle):
        """
        Cancel callback with Context7 documentation integration.
        """
        self.get_logger().info('Received cancel request - checking cancellation best practices')
        
        # Retrieve cancellation best practices
        doc_content = self._get_documentation("ros2.action.cancellation.best_practices")
        if doc_content:
            # Apply cancellation strategy from documentation
            strategy = self._get_cancellation_strategy(doc_content)
            self.get_logger().info(f'Using cancellation strategy: {strategy}')
        
        return CancelResponse.ACCEPT

    async def documented_execute_callback(self, goal_handle):
        """
        Execute callback with Context7 documentation integration.
        """
        self.get_logger().info('Executing goal with documentation guidance')
        
        # Get execution best practices
        action_type = goal_handle.request.type if hasattr(goal_handle.request, 'type') else 'generic'
        doc_topic = f"ros2.action.{action_type}.execution.guidelines"
        
        execution_guidelines = self._get_documentation(doc_topic)
        
        result = DocumentedAction.Result()
        
        try:
            # Apply execution guidelines from documentation
            if execution_guidelines:
                self._apply_execution_guidelines(execution_guidelines)
            
            # Perform the actual execution with documentation-informed approach
            execution_result = await self._execute_with_guidelines(goal_handle, execution_guidelines)
            
            if execution_result['success']:
                goal_handle.succeed()
                result.success = True
                result.execution_time = execution_result['time']
            else:
                goal_handle.abort()
                result.success = False
                result.error_message = execution_result['error']
        
        except Exception as e:
            goal_handle.abort()
            result.success = False
            result.error_message = f"Execution failed: {str(e)}"
        
        return result

    def _get_documentation(self, topic: str) -> Optional[Dict[str, Any]]:
        """
        Get documentation for a topic with caching.
        """
        if topic in self._doc_cache:
            cached_doc = self._doc_cache[topic]
            if time.time() - cached_doc['timestamp'] < self._cache_ttl:
                self._doc_metrics['cache_hits'] += 1
                return cached_doc
        
        # Request from Context7 system (simulated)
        self._request_documentation(topic)
        self._doc_metrics['cache_misses'] += 1
        
        return None

    def _request_documentation(self, topic: str):
        """
        Request documentation from Context7 system.
        """
        request_msg = String()
        request_data = {
            'request_type': 'action_documentation',
            'topic': topic,
            'requester': self.get_name(),
            'timestamp': time.time()
        }
        request_msg.data = json.dumps(request_data)
        self._doc_request_pub.publish(request_msg)
        self._doc_metrics['requests'] += 1

    def _validate_request_with_docs(self, request, doc_content: Dict[str, Any]) -> bool:
        """
        Validate request using guidelines from documentation.
        """
        # In a real implementation, this would parse the documentation
        # and apply validation rules
        return True  # Simplified for example

    def _check_rate_limit_by_docs(self, action_type: str) -> bool:
        """
        Check rate limit based on documentation guidelines.
        """
        # In a real implementation, this would retrieve rate limit information
        # from the documentation and apply it
        return True  # Simplified for example

    def _get_cancellation_strategy(self, doc_content: Dict[str, Any]) -> str:
        """
        Get recommended cancellation strategy from documentation.
        """
        # Parse documentation for recommended strategy
        content = doc_content.get('content', '')
        if "graceful" in content.lower():
            return "graceful"
        elif "immediate" in content.lower():
            return "immediate"
        else:
            return "default"

    def _apply_execution_guidelines(self, doc_content: Dict[str, Any]):
        """
        Apply execution guidelines from documentation.
        """
        # In a real implementation, this would parse execution guidelines
        # from the documentation and configure the action execution accordingly
        content = doc_content.get('content', '')
        
        if "asynchronous" in content.lower():
            self.get_logger().info("Applying asynchronous execution pattern from documentation")
        if "synchronous" in content.lower():
            self.get_logger().info("Applying synchronous execution pattern from documentation")
        if "batch" in content.lower():
            self.get_logger().info("Applying batch processing pattern from documentation")

    async def _execute_with_guidelines(self, goal_handle, guidelines: Optional[Dict[str, Any]]) -> Dict[str, Any]:
        """
        Execute action with documentation-informed approach.
        """
        start_time = time.time()
        
        # Simulate execution following documentation guidelines
        await asyncio.sleep(1.0)  # Simulate actual work
        
        # Determine success based on guidelines (simplified)
        success = True
        error = None
        
        if guidelines:
            content = guidelines.get('content', '')
            if "fail_simulation" in content:
                success = False
                error = "Simulated failure based on documentation"
        
        execution_time = time.time() - start_time
        
        return {
            'success': success,
            'time': execution_time,
            'error': error
        }

    def periodic_doc_update(self):
        """Periodically update documentation for active action types."""
        topics_to_update = [
            "ros2.action.navigation.best_practices",
            "ros2.action.manipulation.best_practices", 
            "ros2.action.calibration.best_practices",
            "ros2.action.performance.optimization"
        ]
        
        for topic in topics_to_update:
            # Only request if cache is expired
            current_time = time.time()
            if topic not in self._doc_cache or (current_time - self._doc_cache[topic]['timestamp']) > self._cache_ttl:
                self._request_documentation(topic)

class ActionDocumentationManager:
    """
    Manager for handling action documentation with Context7 integration.
    """
    
    def __init__(self, node: Node):
        self._node = node
        self._documentation_store = {}
        self._doc_subscription = node.create_subscription(
            String, 'action_documentation_updates', 
            self._handle_doc_update, 10
        )
        
    def _handle_doc_update(self, msg):
        """Handle documentation updates."""
        try:
            doc_data = json.loads(msg.data)
            topic = doc_data.get('topic')
            content = doc_data.get('content')
            
            if topic and content:
                self._documentation_store[topic] = {
                    'content': content,
                    'timestamp': time.time(),
                    'version': doc_data.get('version', '1.0')
                }
                
                self._node.get_logger().info(f'Documentation updated for: {topic}')
        
        except json.JSONDecodeError:
            self._node.get_logger().error('Invalid documentation update format')

def main_context7_integration(args=None):
    """Main function for Context7 integration demonstration."""
    rclpy.init(args=args)
    
    node = Context7ActionIntegrationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Context7 integration node stopped cleanly')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main_context7_integration()
```

## Implementation Best Practices

### Performance Optimization

Performance optimization for ROS 2 Actions involves several key considerations:

1. **Minimize Serialization Overhead**: Use efficient message formats and avoid unnecessarily complex data structures.

2. **Optimize Feedback Frequency**: Balance the need for progress updates with system performance.

3. **Resource Management**: Properly manage memory and computational resources during long-running operations.

4. **QoS Configuration**: Configure Quality of Service settings appropriately for action traffic patterns.

5. **Concurrency Patterns**: Use appropriate threading models to avoid blocking the main ROS execution thread.

### Security Considerations

Security for ROS 2 Actions includes:

1. **Authentication and Authorization**: Verify the identity of action clients and control access.

2. **Data Encryption**: Encrypt sensitive action data in transit and at rest.

3. **Input Validation**: Validate all action requests to prevent injection attacks.

4. **Rate Limiting**: Prevent denial-of-service attacks through rate limiting.

5. **Audit Logging**: Log all action requests and responses for security monitoring.

### Error Handling and Recovery

Robust error handling for Actions should include:

1. **Graceful Degradation**: Continue operating in reduced capacity when possible.

2. **Comprehensive Error Reporting**: Provide detailed error messages to facilitate debugging.

3. **Automatic Recovery**: Implement automatic recovery from transient failures.

4. **Manual Intervention**: Provide mechanisms for manual intervention when automatic recovery fails.

5. **State Persistence**: Preserve state across restarts and failures.

## Advanced Action Patterns

### Stateful Actions

Stateful actions maintain context between successive goal executions, enabling more sophisticated behaviors:

```python
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from action_msgs.msg import GoalStatus
import json
from typing import Dict, Any

class StatefulActionServer(Node):
    """
    Action server that maintains state between goal executions.
    This pattern is useful for actions that build upon previous operations.
    """
    
    def __init__(self):
        super().__init__('stateful_action_server')
        
        self._action_server = ActionServer(
            self,
            StatefulAction,
            'stateful_operation',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )
        
        # Persistent state across goal executions
        self._session_state = {
            'session_id': None,
            'accumulated_data': [],
            'operation_count': 0,
            'cumulative_result': None,
            'preferences': {},
            'history': []
        }
        
        self.get_logger().info('Stateful action server initialized')

    def goal_callback(self, goal_request):
        """Handle goal requests with state considerations."""
        # Validate that this goal type is appropriate given current state
        if goal_request.operation_type == 'reset' and len(self._session_state['accumulated_data']) > 0:
            self.get_logger().info('Session state will be reset by this goal')
        
        # Set up session if needed
        if self._session_state['session_id'] is None:
            import uuid
            self._session_state['session_id'] = str(uuid.uuid4())
        
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Cancel callback that preserves session state."""
        # Cancellations shouldn't affect session state for this example
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        """Execute goal and update persistent state."""
        result = StatefulAction.Result()
        
        try:
            # Perform the operation based on goal type
            operation_result = self._perform_stateful_operation(goal_handle.request)
            
            if operation_result['success']:
                # Update state based on operation result
                self._update_session_state(goal_handle.request, operation_result)
                
                goal_handle.succeed()
                result.success = True
                result.session_state = json.dumps(self._session_state)
            else:
                goal_handle.abort()
                result.success = False
                result.error_message = operation_result['error']
        
        except Exception as e:
            goal_handle.abort()
            result.success = False
            result.error_message = f"Stateful operation failed: {str(e)}"
        
        return result

    def _perform_stateful_operation(self, request) -> Dict[str, Any]:
        """Perform the actual operation considering current state."""
        try:
            if request.operation_type == 'accumulate':
                # Add data to accumulated dataset
                self._session_state['accumulated_data'].append(request.input_data)
                self._session_state['operation_count'] += 1
                
                return {
                    'success': True,
                    'operation_result': f"Added data to accumulation. Total items: {len(self._session_state['accumulated_data'])}"
                }
            
            elif request.operation_type == 'process':
                # Process accumulated data
                if self._session_state['accumulated_data']:
                    # Perform processing on accumulated data
                    processed_result = self._process_accumulated_data()
                    self._session_state['cumulative_result'] = processed_result
                    
                    return {
                        'success': True,
                        'operation_result': f"Processed {len(self._session_state['accumulated_data'])} items. Result: {processed_result}"
                    }
                else:
                    return {
                        'success': False,
                        'error': "No accumulated data to process"
                    }
            
            elif request.operation_type == 'reset':
                # Reset session state
                old_session_id = self._session_state['session_id']
                self._session_state = {
                    'session_id': old_session_id,
                    'accumulated_data': [],
                    'operation_count': 0,
                    'cumulative_result': None,
                    'preferences': self._session_state.get('preferences', {}),
                    'history': self._session_state['history'][-50:]  # Keep last 50 history entries
                }
                
                return {
                    'success': True,
                    'operation_result': "Session state reset"
                }
            
            else:
                return {
                    'success': False,
                    'error': f"Unknown operation type: {request.operation_type}"
                }
        
        except Exception as e:
            return {
                'success': False,
                'error': f"Operation failed: {str(e)}"
            }

    def _update_session_state(self, request, operation_result):
        """Update session state based on completed operation."""
        # Add to operation history
        history_entry = {
            'timestamp': time.time(),
            'operation_type': request.operation_type,
            'input_data': request.input_data,
            'result': operation_result['operation_result'],
            'session_data_size': len(self._session_state['accumulated_data'])
        }
        
        self._session_state['history'].append(history_entry)
        
        # Maintain history size limit
        if len(self._session_state['history']) > 1000:
            self._session_state['history'] = self._session_state['history'][-500:]

    def _process_accumulated_data(self) -> Any:
        """Process the accumulated data."""
        # Perform some computation on accumulated data
        # This is a simplified example
        if self._session_state['accumulated_data']:
            # Example: compute average of numeric values
            numeric_values = [
                float(item) for item in self._session_state['accumulated_data']
                if isinstance(item, (int, float, str)) and str(item).replace('.', '').replace('-', '').isdigit()
            ]
            
            if numeric_values:
                return sum(numeric_values) / len(numeric_values)  # average
        
        return "No numeric data to process"

def main_stateful_action(args=None):
    """Main function for stateful action demonstration."""
    rclpy.init(args=args)
    
    node = StatefulActionServer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Stateful action server stopped cleanly')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main_stateful_action()
```

### Distributed Action Coordination

For complex robotic systems, actions may need to be coordinated across multiple nodes:

```python
import rclpy
from rclpy.action import ActionServer, ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile
from functools import partial
import asyncio
import json
from typing import Dict, Any, List

class DistributedActionCoordinatorNode(Node):
    """
    Node that coordinates multiple related actions across different subsystems.
    This pattern is useful for complex operations that require coordination.
    """
    
    def __init__(self):
        super().__init__('distributed_action_coordinator')
        
        # Create action server for the coordinated operation
        self._coordinated_server = ActionServer(
            self,
            CoordinatedAction,
            'coordinated_operation',
            execute_callback=self.coordinated_execute_callback,
            goal_callback=self.coordinated_goal_callback,
            cancel_callback=self.coordinated_cancel_callback
        )
        
        # Create clients for different subsystems
        self._navigation_client = ActionClient(
            self, NavigationAction, 'navigate_to_pose'
        )
        self._manipulation_client = ActionClient(
            self, ManipulationAction, 'manipulate_object'
        )
        self._perception_client = ActionClient(
            self, PerceptionAction, 'analyze_environment'
        )
        
        # Track coordinated operations
        self._active_operations = {}
        
        self.get_logger().info('Distributed action coordinator initialized')

    def coordinated_goal_callback(self, goal_request):
        """Validate coordinated goal across subsystems."""
        # Check if all required subsystems are available
        clients_available = all([
            self._navigation_client.wait_for_server(timeout_sec=0.1),
            self._manipulation_client.wait_for_server(timeout_sec=0.1),
            self._perception_client.wait_for_server(timeout_sec=0.1)
        ])
        
        if not clients_available:
            self.get_logger().warn('Not all required subsystems available')
            return GoalResponse.REJECT
        
        return GoalResponse.ACCEPT

    def coordinated_cancel_callback(self, goal_handle):
        """Cancel coordinated operation."""
        # In a real implementation, this would propagate cancellation
        # to all participating subsystems
        return CancelResponse.ACCEPT

    async def coordinated_execute_callback(self, goal_handle):
        """Execute coordinated operation across multiple subsystems."""
        result = CoordinatedAction.Result()
        
        # Generate unique operation ID
        operation_id = f"coord_{goal_handle.goal_id.uuid}"
        
        try:
            # Execute coordination sequence
            coordination_result = await self._execute_coordination_sequence(
                goal_handle.request, operation_id
            )
            
            if coordination_result['success']:
                goal_handle.succeed()
                result.success = True
                result.subsystem_results = coordination_result['subsystem_results']
                result.coordination_time = coordination_result['total_time']
            else:
                goal_handle.abort()
                result.success = False
                result.error_message = coordination_result['error']
        
        except Exception as e:
            goal_handle.abort()
            result.success = False
            result.error_message = f"Coordination failed: {str(e)}"
        
        # Clean up operation tracking
        if operation_id in self._active_operations:
            del self._active_operations[operation_id]
        
        return result

    async def _execute_coordination_sequence(self, request, operation_id: str) -> Dict[str, Any]:
        """Execute the coordination sequence across subsystems."""
        start_time = time.time()
        
        # Track results from each subsystem
        subsystem_results = {}
        
        # Step 1: Perception (analyze environment)
        if request.requires_perception:
            perception_result = await self._execute_perception_step(request, operation_id)
            subsystem_results['perception'] = perception_result
            if not perception_result['success']:
                return {'success': False, 'error': f"Perception failed: {perception_result['error']}", 'subsystem_results': subsystem_results}
        
        # Step 2: Navigation (move to position)
        if request.requires_navigation:
            navigation_result = await self._execute_navigation_step(request, operation_id)
            subsystem_results['navigation'] = navigation_result
            if not navigation_result['success']:
                return {'success': False, 'error': f"Navigation failed: {navigation_result['error']}", 'subsystem_results': subsystem_results}
        
        # Step 3: Manipulation (perform action)
        if request.requires_manipulation:
            manipulation_result = await self._execute_manipulation_step(request, operation_id)
            subsystem_results['manipulation'] = manipulation_result
            if not manipulation_result['success']:
                return {'success': False, 'error': f"Manipulation failed: {manipulation_result['error']}", 'subsystem_results': subsystem_results}
        
        total_time = time.time() - start_time
        
        return {
            'success': True,
            'subsystem_results': subsystem_results,
            'total_time': total_time
        }

    async def _execute_perception_step(self, request, operation_id: str) -> Dict[str, Any]:
        """Execute perception step."""
        try:
            goal_msg = PerceptionAction.Goal()
            goal_msg.scan_area = request.perception_area if hasattr(request, 'perception_area') else [0.0, 0.0, 1.0, 1.0]
            goal_msg.object_types = request.required_objects if hasattr(request, 'required_objects') else []
            
            # Wait for server and send goal
            self._perception_client.wait_for_server()
            future = self._perception_client.send_goal_async(goal_msg)
            goal_handle = await future
            
            if not goal_handle.accepted:
                return {'success': False, 'error': 'Perception goal rejected'}
            
            result_future = goal_handle.get_result_async()
            result_data = await result_future
            
            return {
                'success': result_data.result.success,
                'result': result_data.result if result_data.result.success else None,
                'error': result_data.result.error_message if not result_data.result.success else None
            }
        
        except Exception as e:
            return {'success': False, 'error': f"Perception step failed: {str(e)}"}

    async def _execute_navigation_step(self, request, operation_id: str) -> Dict[str, Any]:
        """Execute navigation step."""
        try:
            goal_msg = NavigationAction.Goal()
            goal_msg.target_pose = request.navigation_target if hasattr(request, 'navigation_target') else None
            goal_msg.path_constraints = request.path_constraints if hasattr(request, 'path_constraints') else {}
            
            # Wait for server and send goal
            self._navigation_client.wait_for_server()
            future = self._navigation_client.send_goal_async(goal_msg)
            goal_handle = await future
            
            if not goal_handle.accepted:
                return {'success': False, 'error': 'Navigation goal rejected'}
            
            result_future = goal_handle.get_result_async()
            result_data = await result_future
            
            return {
                'success': result_data.result.success,
                'result': result_data.result if result_data.result.success else None,
                'error': result_data.result.error_message if not result_data.result.success else None
            }
        
        except Exception as e:
            return {'success': False, 'error': f"Navigation step failed: {str(e)}"}

    async def _execute_manipulation_step(self, request, operation_id: str) -> Dict[str, Any]:
        """Execute manipulation step."""
        try:
            goal_msg = ManipulationAction.Goal()
            goal_msg.target_object = request.manipulation_target if hasattr(request, 'manipulation_target') else None
            goal_msg.operation_type = request.manipulation_operation if hasattr(request, 'manipulation_operation') else 'grasp'
            
            # Wait for server and send goal
            self._manipulation_client.wait_for_server()
            future = self._manipulation_client.send_goal_async(goal_msg)
            goal_handle = await future
            
            if not goal_handle.accepted:
                return {'success': False, 'error': 'Manipulation goal rejected'}
            
            result_future = goal_handle.get_result_async()
            result_data = await result_future
            
            return {
                'success': result_data.result.success,
                'result': result_data.result if result_data.result.success else None,
                'error': result_data.result.error_message if not result_data.result.success else None
            }
        
        except Exception as e:
            return {'success': False, 'error': f"Manipulation step failed: {str(e)}"}

def main_distributed_action(args=None):
    """Main function for distributed action coordination."""
    rclpy.init(args=args)
    
    node = DistributedActionCoordinatorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Distributed action coordinator stopped cleanly')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main_distributed_action()
```

### Real-World Examples

Here's a practical example of Actions used in a mobile robot for navigation:

```python
import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from tf2_ros import TransformListener, Buffer
import tf2_geometry_msgs
import math

class NavigationActionServer(Node):
    """
    Practical navigation action server for mobile robots.
    Implements real-world navigation with obstacle avoidance and path following.
    """
    
    def __init__(self):
        super().__init__('navigation_action_server')
        
        # Create navigation action server
        self._nav_action_server = ActionServer(
            self,
            NavigateToPose,
            'navigate_to_pose',
            execute_callback=self.navigate_execute_callback,
            goal_callback=self.navigate_goal_callback,
            cancel_callback=self.navigate_cancel_callback
        )
        
        # Publishers and subscribers
        self._path_pub = self.create_publisher(Path, 'current_path', 10)
        self._cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # TF2 for transformations
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        
        # Navigation state
        self._current_pose = None
        self._navigation_active = False
        self._navigation_goal = None
        
        self.get_logger().info('Navigation action server initialized')

    def navigate_goal_callback(self, goal_request):
        """Validate navigation goal."""
        target_pose = goal_request.pose
        
        # Check if target is within reasonable bounds
        if abs(target_pose.position.x) > 100 or abs(target_pose.position.y) > 100:
            self.get_logger().warn('Navigation goal is outside reasonable bounds')
            return GoalResponse.REJECT
        
        # Check if navigation is currently active (only one goal at a time for this example)
        if self._navigation_active:
            self.get_logger().warn('Navigation already active, rejecting new goal')
            return GoalResponse.REJECT
        
        return GoalResponse.ACCEPT

    def navigate_cancel_callback(self, goal_handle):
        """Handle navigation cancellation."""
        self.get_logger().info('Navigation cancellation requested')
        return CancelResponse.ACCEPT

    async def navigate_execute_callback(self, goal_handle):
        """Execute navigation to the specified pose."""
        self.get_logger().info('Starting navigation to pose')
        
        result = NavigateToPose.Result()
        feedback_msg = NavigateToPose.Feedback()
        
        # Store goal info
        self._navigation_active = True
        self._navigation_goal = goal_handle.request.pose
        start_pose = self._get_current_pose()
        
        if start_pose is None:
            result.error_code = NavigateToPose.Result.FAILURE
            result.error_message = "Unable to determine current pose"
            goal_handle.abort()
            self._navigation_active = False
            return result
        
        # Plan path to goal
        path = self._plan_path(start_pose, goal_handle.request.pose)
        if not path:
            result.error_code = NavigateToPose.Result.FAILURE
            result.error_message = "Unable to find valid path to goal"
            goal_handle.abort()
            self._navigation_active = False
            return result
        
        # Publish path
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.poses = path
        self._path_pub.publish(path_msg)
        
        # Follow path
        success = True
        for i, pose in enumerate(path.poses):
            # Check for cancellation
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Navigation cancelled during execution')
                result.error_code = NavigateToPose.Result.CANCELED
                goal_handle.canceled()
                success = False
                break
            
            # Move to this pose in the path
            reached_pose = await self._move_to_pose(pose, goal_handle)
            
            if not reached_pose:
                self.get_logger().error(f'Failed to reach path pose {i+1}/{len(path.poses)}')
                success = False
                break
            
            # Update feedback
            distance_remaining = self._euclidean_distance(
                self._get_current_pose(), goal_handle.request.pose
            )
            progress = ((i + 1) / len(path.poses)) * 100.0
            
            feedback_msg.current_pose = self._get_current_pose()
            feedback_msg.distance_remaining = distance_remaining
            feedback_msg.progress = progress
            goal_handle.publish_feedback(feedback_msg)
        
        if success:
            # Final check if we're at the goal
            current_pose = self._get_current_pose()
            if current_pose:
                final_distance = self._euclidean_distance(current_pose, goal_handle.request.pose)
                goal_tolerance = goal_handle.request.tolerance or 0.5
                
                if final_distance <= goal_tolerance:
                    result.error_code = NavigateToPose.Result.SUCCEEDED
                    goal_handle.succeed()
                    self.get_logger().info(f'Navigation succeeded. Final distance from goal: {final_distance:.2f}m')
                else:
                    result.error_code = NavigateToPose.Result.FAILURE
                    result.error_message = f"Close to path end but not at goal: {final_distance:.2f}m from goal"
                    goal_handle.abort()
            else:
                result.error_code = NavigateToPose.Result.FAILURE
                result.error_message = "Unable to determine final pose"
                goal_handle.abort()
        else:
            result.error_code = NavigateToPose.Result.FAILURE
            result.error_message = "Navigation failed during path execution"
            goal_handle.abort()
        
        self._navigation_active = False
        self._navigation_goal = None
        
        return result

    def _plan_path(self, start_pose, goal_pose):
        """Plan a simple path from start to goal (in a real system, use path planning algorithms)."""
        # Simplified path planning - in reality would use A*, RRT, or other algorithms
        path = Path()
        path.header.frame_id = 'map'
        
        # Create a straight-line path (simplified)
        steps = 10
        for i in range(steps + 1):
            alpha = i / steps
            intermediate_pose = PoseStamped()
            intermediate_pose.header.frame_id = 'map'
            intermediate_pose.pose.position.x = start_pose.position.x + alpha * (goal_pose.position.x - start_pose.position.x)
            intermediate_pose.pose.position.y = start_pose.position.y + alpha * (goal_pose.position.y - start_pose.position.y)
            # Simple orientation interpolation would go here
            
            path.poses.append(intermediate_pose)
        
        return path

    async def _move_to_pose(self, pose, goal_handle):
        """Move to a specific pose in the path."""
        # In a real system, this would implement path following
        # For this example, we'll simulate movement
        
        # Calculate required movement
        current_pose = self._get_current_pose()
        if not current_pose:
            return False
        
        required_x = pose.pose.position.x - current_pose.position.x
        required_y = pose.pose.position.y - current_pose.position.y
        distance = math.sqrt(required_x**2 + required_y**2)
        
        if distance < 0.1:  # Already close enough
            return True
        
        # Simulate movement (in real system, would send commands to base controller)
        duration = distance / 0.5  # Assume 0.5 m/s speed for simulation
        steps = int(duration / 0.1)  # 0.1 second steps
        
        for step in range(steps):
            if goal_handle.is_cancel_requested:
                return False
            
            # Simulate moving toward the pose
            await asyncio.sleep(0.1)
        
        return True

    def _get_current_pose(self):
        """Get current robot pose from localization system (simplified)."""
        try:
            # Look up transform from base_link to map frame
            t = self._tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time()
            )
            
            pose = Pose()
            pose.position.x = t.transform.translation.x
            pose.position.y = t.transform.translation.y
            pose.position.z = t.transform.translation.z
            pose.orientation = t.transform.rotation
            
            return pose
        except:
            # In simulation or if TF isn't available, return None or default pose
            return None

    def _euclidean_distance(self, pose1, pose2):
        """Calculate Euclidean distance between two poses."""
        dx = pose2.position.x - pose1.position.x
        dy = pose2.position.y - pose1.position.y
        return math.sqrt(dx*dx + dy*dy)

def main_navigation_action(args=None):
    """Main function for navigation action server."""
    rclpy.init(args=args)
    
    node = NavigationActionServer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Navigation action server stopped cleanly')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main_navigation_action()
```

## Testing and Debugging

### Testing Strategies

Testing ROS 2 Actions requires specialized approaches:

1. **Unit Testing**: Test individual action components in isolation
2. **Integration Testing**: Test action server-client interactions
3. **System Testing**: Test complete action workflows
4. **Performance Testing**: Test action throughput and latency under load
5. **Stress Testing**: Test behavior under extreme conditions

### Debugging Techniques

Effective debugging of Actions includes:

1. **Action State Monitoring**: Monitor action state transitions
2. **Feedback Stream Analysis**: Analyze feedback messages for progress issues
3. **Result Evaluation**: Verify result correctness and completeness
4. **Cancellation Testing**: Ensure proper cancellation handling
5. **Error Injection**: Test error handling by injecting failures

## Summary

ROS 2 Actions represent a sophisticated communication pattern that enables sophisticated robotic behaviors requiring long-running operations with feedback, cancellation, and detailed results. The pattern addresses limitations of simple services while providing more control than topics alone.

The integration of Context7 documentation systems enhances the development process by providing immediate access to up-to-date best practices, API references, and implementation guidelines. This integration streamlines development workflows and ensures implementations align with current standards.

Key aspects of effective Action implementation include:
- Proper state machine design for action lifecycle management
- Appropriate Quality of Service configuration
- Efficient feedback mechanisms for progress monitoring
- Robust error handling and recovery procedures
- Security considerations for protected operations
- Performance optimization for real-time requirements
- Context7 integration for dynamic documentation access

The evolution from ROS 1 to ROS 2 Actions brings significant improvements in decentralization, security, and real-time performance, making them suitable for safety-critical robotic applications.

Through careful application of the patterns and practices outlined in this guide, developers can create robust, efficient, and maintainable action-based robotic systems that leverage the full potential of ROS 2's distributed architecture.