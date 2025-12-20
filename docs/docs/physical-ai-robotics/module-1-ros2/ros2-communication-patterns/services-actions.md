---
sidebar_position: 3
sidebar_label: Services and Actions - Advanced Communication Patterns
---

# Services and Actions - Advanced Communication Patterns in ROS 2

## Introduction to Advanced Communication Patterns

While topics provide the foundation for asynchronous data streaming in ROS 2, many robotic applications require more structured communication patterns to handle complex operations that involve request-response cycles and long-running tasks. Services and actions are two advanced communication patterns that complement the publish-subscribe model, enabling sophisticated robotic behaviors and system interactions.

Services provide synchronous request-reply communication, ideal for operations that require immediate acknowledgment and results. Actions, on the other hand, extend the service pattern to handle long-running operations with continuous feedback, goal preemption, and structured result reporting. Together, these communication patterns provide the tools necessary to build robust, responsive robotic applications.

The introduction of Quality of Service (QoS) policies in ROS 2 has enhanced these communication patterns with configurable reliability, durability, and delivery guarantees. This allows developers to fine-tune communication behavior based on application requirements, from reliable command delivery to best-effort sensor data transmission.

## Deep Technical Analysis of Services

### Service Architecture and Implementation

Services in ROS 2 follow a client-server architecture where service servers provide functionality that can be accessed by one or more service clients. The service interface is defined using Interface Definition Language (IDL) files that specify the request and response message structures.

The service communication cycle consists of several phases:
1. **Request Phase**: Client sends a request message to the server
2. **Processing Phase**: Server processes the request and generates a response
3. **Response Phase**: Server sends the response back to the client
4. **Completion Phase**: Client processes the response and continues execution

This synchronous nature means the client thread blocks until the server responds, making services suitable for operations that require guaranteed delivery and response.

### Service Design Patterns

Different service design patterns address specific use cases in robotic applications:

#### 1. State Query Services
These services provide instantaneous state information without side effects:

```python
# StateQuery.srv
# Request: Empty
# Response: bool available
#           string status
#           float64[] joint_positions
```

```cpp
// StateQuery.srv implementation
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/trigger.hpp"
#include "std_msgs/msg/string.hpp"
#include <mutex>

class StateQueryService : public rclcpp::Node
{
public:
    StateQueryService() : Node("state_query_service")
    {
        // Create state query service
        state_service_ = this->create_service<example_interfaces::srv::Trigger>(
            "query_robot_state",
            std::bind(&StateQueryService::handle_state_query, this, 
                     std::placeholders::_1, std::placeholders::_2)
        );
        
        // Initialize robot state
        update_robot_state();
        
        // Timer to periodically update state
        state_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&StateQueryService::update_robot_state, this)
        );
    }

private:
    void handle_state_query(
        const std::shared_ptr<example_interfaces::srv::Trigger::Request> request,
        std::shared_ptr<example_interfaces::srv::Trigger::Response> response)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        
        // Check if robot is available
        if (robot_available_) {
            response->success = true;
            response->message = build_status_message();
        } else {
            response->success = false;
            response->message = "Robot unavailable - system error";
        }
        
        RCLCPP_INFO(this->get_logger(), "State query responded: %s", response->message.c_str());
    }
    
    void update_robot_state()
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        
        // Simulate state updates based on sensor data or other inputs
        static int counter = 0;
        counter++;
        
        // Calculate availability based on simulated conditions
        robot_available_ = (counter % 10 != 0);  // Robot unavailable every 10th cycle
        
        // Update other state variables
        last_update_timestamp_ = this->get_clock()->now();
        operational_mode_ = (robot_available_) ? "ACTIVE" : "ERROR";
    }
    
    std::string build_status_message()
    {
        std::stringstream ss;
        ss << "Status: " << operational_mode_
           << ", Last Update: " << last_update_timestamp_.seconds()
           << ", Availability: " << (robot_available_ ? "YES" : "NO");
        return ss.str();
    }
    
    rclcpp::Service<example_interfaces::srv::Trigger>::SharedPtr state_service_;
    rclcpp::TimerBase::SharedPtr state_timer_;
    std::mutex state_mutex_;
    
    bool robot_available_ = true;
    std::string operational_mode_ = "IDLE";
    rclcpp::Time last_update_timestamp_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StateQueryService>());
    rclcpp::shutdown();
    return 0;
}
```

#### 2. Command Execution Services
These services perform actions and return success/failure indicators:

```cpp
// CommandExecution.srv
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <functional>
#include <thread>
#include <atomic>

class CommandExecutionService : public rclcpp::Node
{
public:
    CommandExecutionService() : Node("command_execution_service")
    {
        // Create command execution services
        move_service_ = this->create_service<std_srvs::srv::SetBool>(
            "execute_move_command",
            std::bind(&CommandExecutionService::handle_move_command, this,
                     std::placeholders::_1, std::placeholders::_2)
        );
        
        rotate_service_ = this->create_service<std_srvs::srv::SetBool>(
            "execute_rotate_command",
            std::bind(&CommandExecutionService::handle_rotate_command, this,
                     std::placeholders::_1, std::placeholders::_2)
        );
        
        // Create publisher for motion commands
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "cmd_vel", 10
        );
        
        RCLCPP_INFO(this->get_logger(), "Command execution service initialized");
    }

private:
    void handle_move_command(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        if (current_operation_active_.load()) {
            response->success = false;
            response->message = "Another operation in progress";
            RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
            return;
        }
        
        current_operation_active_.store(true);
        
        try {
            // Create motion command
            auto cmd_msg = geometry_msgs::msg::Twist();
            cmd_msg.linear.x = (request->data) ? 0.5 : 0.0;  // Move forward if true, stop if false
            cmd_msg.angular.z = 0.0;
            
            // Execute command
            cmd_vel_publisher_->publish(cmd_msg);
            
            // Simulate execution time
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            
            response->success = true;
            response->message = "Move command executed";
            RCLCPP_INFO(this->get_logger(), "Move command: %s", response->message.c_str());
            
        } catch (const std::exception& e) {
            response->success = false;
            response->message = std::string("Move command failed: ") + e.what();
            RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        }
        
        current_operation_active_.store(false);
    }
    
    void handle_rotate_command(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        if (current_operation_active_.load()) {
            response->success = false;
            response->message = "Another operation in progress";
            RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
            return;
        }
        
        current_operation_active_.store(true);
        
        try {
            // Create rotation command
            auto cmd_msg = geometry_msgs::msg::Twist();
            cmd_msg.linear.x = 0.0;
            cmd_msg.angular.z = (request->data) ? 0.5 : 0.0;  // Rotate if true, stop if false
            
            // Execute command
            cmd_vel_publisher_->publish(cmd_msg);
            
            // Simulate execution time
            std::this_thread::sleep_for(std::chrono::milliseconds(750));
            
            response->success = true;
            response->message = "Rotation command executed";
            RCLCPP_INFO(this->get_logger(), "Rotation command: %s", response->message.c_str());
            
        } catch (const std::exception& e) {
            response->success = false;
            response->message = std::string("Rotation command failed: ") + e.what();
            RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        }
        
        current_operation_active_.store(false);
    }
    
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr move_service_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr rotate_service_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    
    std::atomic<bool> current_operation_active_{false};
};
```

### Advanced Service Implementation

#### Service with Timeout and Retry Logic

```python
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from std_srvs.srv import SetBool
from example_interfaces.srv import Trigger
import time
from typing import Tuple, Optional
import threading
from functools import wraps

def retry_service(max_attempts: int, delay: float):
    """Decorator to add retry logic to service calls"""
    def decorator(func):
        @wraps(func)
        def wrapper(*args, **kwargs):
            for attempt in range(max_attempts):
                try:
                    return func(*args, **kwargs)
                except Exception as e:
                    if attempt == max_attempts - 1:
                        raise e
                    time.sleep(delay)
            return None
        return wrapper
    return decorator

class RobustServiceServer(Node):
    def __init__(self):
        super().__init__('robust_service_server')
        
        # Create service with callback group
        service_cb_group = MutuallyExclusiveCallbackGroup()
        
        self.health_check_service = self.create_service(
            Trigger,
            'health_check',
            self.health_check_callback,
            callback_group=service_cb_group
        )
        
        self.calibration_service = self.create_service(
            SetBool,
            'calibration',
            self.calibration_callback,
            callback_group=service_cb_group
        )
        
        # Service statistics
        self.service_stats = {
            'health_check': {'calls': 0, 'successes': 0, 'failures': 0},
            'calibration': {'calls': 0, 'successes': 0, 'failures': 0}
        }
        
        # Simulate system components
        self.components = {
            'sensors': True,
            'motors': True,
            'processors': True,
            'network': True
        }
        
        # Service response delay simulation
        self.response_delay = 0.1
        
        self.get_logger().info('Robust service server initialized')
    
    def health_check_callback(self, request, response):
        """Perform comprehensive health check"""
        self.service_stats['health_check']['calls'] += 1
        
        start_time = time.time()
        
        try:
            # Simulate health check process
            time.sleep(self.response_delay)
            
            # Check all system components
            all_healthy = all(self.components.values())
            
            if all_healthy:
                response.success = True
                response.message = self.build_health_report()
                self.service_stats['health_check']['successes'] += 1
                self.get_logger().info(f'Health check succeeded: {response.message}')
            else:
                response.success = False
                response.message = self.build_error_report()
                self.service_stats['health_check']['failures'] += 1
                self.get_logger().warn(f'Health check failed: {response.message}')
                
        except Exception as e:
            response.success = False
            response.message = f'Health check failed with exception: {str(e)}'
            self.service_stats['health_check']['failures'] += 1
            self.get_logger().error(f'Health check exception: {response.message}')
        
        duration = time.time() - start_time
        self.get_logger().debug(f'Health check took {duration:.3f}s')
        
        return response
    
    def calibration_callback(self, request, response):
        """Handle calibration requests"""
        self.service_stats['calibration']['calls'] += 1
        
        start_time = time.time()
        
        try:
            if request.data:
                # Start calibration
                self.get_logger().info('Starting calibration process...')
                
                # Simulate calibration process
                time.sleep(1.5)  # Simulate real calibration time
                
                # Check if calibration was successful
                success = self.perform_calibration()
                
                if success:
                    response.success = True
                    response.message = 'Calibration completed successfully'
                    self.service_stats['calibration']['successes'] += 1
                    self.get_logger().info(response.message)
                else:
                    response.success = False
                    response.message = 'Calibration failed - check sensor configuration'
                    self.service_stats['calibration']['failures'] += 1
                    self.get_logger().error(response.message)
            else:
                # Reset to known state
                self.reset_system()
                response.success = True
                response.message = 'System reset to calibrated state'
                self.service_stats['calibration']['successes'] += 1
                self.get_logger().info(response.message)
                
        except Exception as e:
            response.success = False
            response.message = f'Calibration failed with exception: {str(e)}'
            self.service_stats['calibration']['failures'] += 1
            self.get_logger().error(f'Calibration exception: {response.message}')
        
        duration = time.time() - start_time
        self.get_logger().debug(f'Calibration took {duration:.3f}s')
        
        return response
    
    def perform_calibration(self) -> bool:
        """Simulate calibration process"""
        # Simulate potential calibration failure (10% chance)
        import random
        return random.random() > 0.1
    
    def reset_system(self):
        """Reset system to calibrated state"""
        # Reset internal states
        pass
    
    def build_health_report(self) -> str:
        """Build comprehensive health report"""
        healthy_comps = [name for name, healthy in self.components.items() if healthy]
        return f'System health OK - {len(healthy_comps)}/{len(self.components)} components operational'
    
    def build_error_report(self) -> str:
        """Build error report for unhealthy components"""
        unhealthy_comps = [name for name, healthy in self.components.items() if not healthy]
        return f'System error - {len(unhealthy_comps)} components unhealthy: {", ".join(unhealthy_comps)}'
    
    def get_service_statistics(self) -> dict:
        """Retrieve service statistics"""
        return self.service_stats.copy()

class ServiceClientWithTimeout(Node):
    def __init__(self):
        super().__init__('service_client_with_timeout')
        
        # Create clients for services
        self.health_check_client = self.create_client(Trigger, 'health_check')
        self.calibration_client = self.create_client(SetBool, 'calibration')
        
        # Wait for services with timeout
        self.get_logger().info('Waiting for services...')
        self.wait_for_services()
        
        # Test service calls
        self.test_services()
    
    def wait_for_services(self):
        """Wait for services with timeout"""
        timeout = 5.0  # seconds
        start_time = time.time()
        
        while not self.health_check_client.wait_for_service(timeout_sec=1.0):
            if time.time() - start_time > timeout:
                self.get_logger().error('Health check service not available after timeout')
                break
            self.get_logger().info('Waiting for health_check service...')
        
        start_time = time.time()
        while not self.calibration_client.wait_for_service(timeout_sec=1.0):
            if time.time() - start_time > timeout:
                self.get_logger().error('Calibration service not available after timeout')
                break
            self.get_logger().info('Waiting for calibration service...')
    
    def test_services(self):
        """Test service calls with timeout and error handling"""
        self.get_logger().info('Testing services with timeout...')
        
        # Test health check
        self.call_health_check_with_timeout()
        time.sleep(1)
        
        # Test calibration
        self.call_calibration_with_timeout()
    
    def call_health_check_with_timeout(self):
        """Call health check service with timeout"""
        request = Trigger.Request()
        
        # Use asyncio for timeout
        import asyncio
        from concurrent.futures import ThreadPoolExecutor
        
        async def call_service():
            future = self.health_check_client.call_async(request)
            
            try:
                # Wait for response with timeout
                response = await asyncio.wait_for(
                    asyncio.wrap_future(future), 
                    timeout=3.0  # 3 second timeout
                )
                return response
            except asyncio.TimeoutError:
                self.get_logger().error('Health check service call timed out')
                return None
        
        # Run the async call in the event loop
        try:
            # For ROS 2, we need to use a different approach for async calls
            future = self.health_check_client.call_async(request)
            
            # Use executor to wait with timeout
            start_time = time.time()
            while not future.done():
                if time.time() - start_time > 3.0:  # 3 second timeout
                    self.get_logger().error('Health check service call timed out')
                    return
                
                rclpy.spin_once(self, timeout_sec=0.1)
            
            response = future.result()
            self.get_logger().info(f'Health check response: {response.success}, {response.message}')
            
        except Exception as e:
            self.get_logger().error(f'Health check call failed: {str(e)}')
    
    def call_calibration_with_timeout(self):
        """Call calibration service with timeout"""
        request = SetBool.Request()
        request.data = True  # Start calibration
        
        future = self.calibration_client.call_async(request)
        
        start_time = time.time()
        while not future.done():
            if time.time() - start_time > 5.0:  # 5 second timeout
                self.get_logger().error('Calibration service call timed out')
                return
            
            rclpy.spin_once(self, timeout_sec=0.1)
        
        response = future.result()
        self.get_logger().info(f'Calibration response: {response.success}, {response.message}')

def run_service_with_timeout_example():
    """Run service with timeout example"""
    rclpy.init()
    
    # Create server and client
    server = RobustServiceServer()
    client = ServiceClientWithTimeout()
    
    # Create executor
    executor = SingleThreadedExecutor()
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
    run_service_with_timeout_example()
```

## Deep Technical Analysis of Actions

### Action Architecture and Implementation

Actions represent the most sophisticated communication pattern in ROS 2, designed specifically for long-running operations that require continuous feedback, goal preemption, and structured result reporting. Unlike services, which are synchronous and blocking, actions allow for asynchronous execution with ongoing interaction between the client and server.

The action interface is defined using IDL files that specify three message types:
1. **Goal**: Defines the desired outcome and optional parameters
2. **Feedback**: Provides ongoing status updates during execution
3. **Result**: Contains the final outcome and optional completion data

### Action Server Implementation

```cpp
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/float64.hpp"
#include <memory>
#include <vector>
#include <algorithm>
#include <cmath>

class AdvancedActionServer : public rclcpp::Node
{
public:
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    using GoalHandleFollowJointTrajectory = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

    explicit AdvancedActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("advanced_action_server", options)
    {
        // Create action server with custom callbacks
        this->action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
            this->get_node_base_interface(),
            this->get_node_clock_interface(),
            this->get_node_logging_interface(),
            this->get_node_waitables_interface(),
            "follow_joint_trajectory",
            std::bind(&AdvancedActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&AdvancedActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&AdvancedActionServer::handle_accepted, this, std::placeholders::_1)
        );
        
        // Create publishers for status and feedback
        status_pub_ = this->create_publisher<std_msgs::msg::Float64>("action_status", 10);
        joint_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectoryPoint>("joint_states", 10);
        
        RCLCPP_INFO(this->get_logger(), "Advanced action server initialized");
    }

private:
    rclcpp_action::Server<FollowJointTrajectory>::SharedPtr action_server_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr status_pub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr joint_pub_;
    
    // Active goals tracking
    std::vector<GoalHandleFollowJointTrajectory::SharedPtr> active_goals_;
    std::mutex goals_mutex_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const FollowJointTrajectory::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request with order %zu", goal->trajectory.points.size());
        
        // Validate goal
        if (goal->trajectory.points.empty()) {
            RCLCPP_WARN(this->get_logger(), "Rejected goal request with empty trajectory");
            return rclcpp_action::GoalResponse::REJECT;
        }
        
        // Check if we can accept another goal
        std::lock_guard<std::mutex> lock(goals_mutex_);
        if (active_goals_.size() >= 1) {  // Limit to 1 concurrent goal
            RCLCPP_WARN(this->get_logger(), "Rejected goal request - too many active goals");
            return rclcpp_action::GoalResponse::REJECT;
        }
        
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received cancel request");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle)
    {
        using namespace std::placeholders;
        
        // Add to active goals
        {
            std::lock_guard<std::mutex> lock(goals_mutex_);
            active_goals_.push_back(goal_handle);
        }
        
        // Execute in separate thread to avoid blocking the executor
        std::thread{std::bind(&AdvancedActionServer::execute, this, _1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal...");
        
        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<FollowJointTrajectory::Result>();
        
        // Get trajectory points
        const auto & trajectory_points = goal->trajectory.points;
        const auto & joint_names = goal->trajectory.joint_names;
        
        double total_duration = 0.0;
        for (const auto & point : trajectory_points) {
            total_duration = std::max(total_duration, 
                point.time_from_start.sec + point.time_from_start.nanosec / 1e9);
        }
        
        // Execute trajectory
        for (size_t i = 0; i < trajectory_points.size(); ++i) {
            // Check if goal was canceled
            if (goal_handle->is_canceling()) {
                RCLCPP_INFO(this->get_logger(), "Goal was canceled");
                
                // Clear active goals
                std::lock_guard<std::mutex> lock(goals_mutex_);
                auto it = std::find(active_goals_.begin(), active_goals_.end(), goal_handle);
                if (it != active_goals_.end()) {
                    active_goals_.erase(it);
                }
                
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal canceled and result sent");
                return;
            }
            
            const auto & point = trajectory_points[i];
            
            // Send feedback
            auto feedback = std::make_shared<FollowJointTrajectory::Feedback>();
            feedback->actual.positions = point.positions;
            feedback->desired.positions = point.positions;
            feedback->error.positions.resize(point.positions.size(), 0.0);
            
            // Calculate progress
            double progress = static_cast<double>(i + 1) / trajectory_points.size();
            auto status_msg = std_msgs::msg::Float64();
            status_msg.data = progress * 100.0;
            status_pub_->publish(status_msg);
            
            goal_handle->publish_feedback(feedback);
            
            // Publish joint states
            auto joint_state_msg = trajectory_msgs::msg::JointTrajectoryPoint();
            joint_state_msg.positions = point.positions;
            joint_state_msg.velocities = point.velocities;
            joint_state_msg.accelerations = point.accelerations;
            joint_pub_->publish(joint_state_msg);
            
            // Sleep to simulate execution time
            auto sleep_time = std::chrono::duration<double>(
                point.time_from_start.sec + point.time_from_start.nanosec / 1e9);
            std::this_thread::sleep_for(sleep_time);
        }
        
        // Check if goal was canceled (final check)
        if (goal_handle->is_canceling()) {
            RCLCPP_INFO(this->get_logger(), "Goal was canceled at the end");
            
            std::lock_guard<std::mutex> lock(goals_mutex_);
            auto it = std::find(active_goals_.begin(), active_goals_.end(), goal_handle);
            if (it != active_goals_.end()) {
                active_goals_.erase(it);
            }
            
            goal_handle->canceled(result);
            RCLCPP_INFO(this->get_logger(), "Goal canceled at the end");
            return;
        }
        
        // Goal completed successfully
        {
            std::lock_guard<std::mutex> lock(goals_mutex_);
            auto it = std::find(active_goals_.begin(), active_goals_.end(), goal_handle);
            if (it != active_goals_.end()) {
                active_goals_.erase(it);
            }
        }
        
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
};

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(AdvancedActionServer)
```

### Advanced Action Client Implementation

```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from std_msgs.msg import Float64
import time
import threading
from typing import List, Optional
import numpy as np

class AdvancedActionClient(Node):
    def __init__(self):
        super().__init__('advanced_action_client')
        
        # Create action client
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            'follow_joint_trajectory'
        )
        
        # Create subscriber for feedback
        self.feedback_sub = self.create_subscription(
            Float64,
            'action_status',
            self.feedback_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.VOLATILE)
        )
        
        self.current_progress = 0.0
        self.action_active = False
        
        # Wait for action server
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('Action server available')
    
    def feedback_callback(self, msg):
        """Handle feedback messages"""
        self.current_progress = msg.data
        if self.action_active:
            self.get_logger().debug(f'Action progress: {self.current_progress:.1f}%')
    
    def send_goal_async(self, joint_names: List[str], trajectory_points: List[JointTrajectoryPoint]):
        """Send goal asynchronously with feedback and result"""
        
        # Build goal message
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = JointTrajectory()
        goal_msg.trajectory.joint_names = joint_names
        goal_msg.trajectory.points = trajectory_points
        
        # Send goal and get future
        self.action_active = True
        self.get_logger().info('Sending goal to action server')
        
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback_wrapper
        )
        
        # Add done callback
        send_goal_future.add_done_callback(self.goal_response_callback)
        
        return send_goal_future
    
    def feedback_callback_wrapper(self, feedback_msg):
        """Wrapper for feedback callback"""
        self.get_logger().debug(f'Received feedback: {feedback_msg}')
    
    def goal_response_callback(self, future):
        """Handle goal response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected by server')
            self.action_active = False
            return
        
        self.get_logger().info('Goal accepted by server')
        
        # Get result future
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        """Handle result callback"""
        result = future.result().result
        self.get_logger().info(f'Action completed with result: {result}')
        self.action_active = False

class TrajectoryGenerator:
    """Utility class to generate different types of trajectories"""
    
    @staticmethod
    def generate_sinusoidal_trajectory(joint_names: List[str], duration: float = 5.0, steps: int = 50) -> List[JointTrajectoryPoint]:
        """Generate sinusoidal trajectory"""
        points = []
        time_step = duration / steps
        
        for i in range(steps + 1):
            t = i * time_step
            
            # Generate sinusoidal positions for each joint
            positions = []
            for j, joint_name in enumerate(joint_names):
                frequency = (j + 1) * 0.5  # Different frequency for each joint
                amplitude = 1.0
                phase = j * 0.5  # Phase offset
                position = amplitude * np.sin(2 * np.pi * frequency * t + phase)
                positions.append(float(position))
            
            # Calculate velocities (derivative of position)
            velocities = []
            for j, joint_name in enumerate(joint_names):
                frequency = (j + 1) * 0.5
                amplitude = 1.0
                phase = j * 0.5
                velocity = amplitude * 2 * np.pi * frequency * np.cos(2 * np.pi * frequency * t + phase)
                velocities.append(float(velocity))
            
            # Create trajectory point
            point = JointTrajectoryPoint()
            point.positions = positions
            point.velocities = velocities
            point.accelerations = [0.0] * len(positions)  # Simplified
            point.time_from_start = Duration(sec=int(time_step * i), nanosec=int((time_step * i - int(time_step * i)) * 1e9))
            
            points.append(point)
        
        return points
    
    @staticmethod
    def generate_linear_trajectory(joint_names: List[str], start_positions: List[float], end_positions: List[float], duration: float = 5.0, steps: int = 100) -> List[JointTrajectoryPoint]:
        """Generate linear interpolation trajectory"""
        points = []
        time_step = duration / steps
        
        for i in range(steps + 1):
            t = i / steps  # Normalized time from 0 to 1
            
            # Interpolate positions
            positions = [
                start_pos + t * (end_pos - start_pos) 
                for start_pos, end_pos in zip(start_positions, end_positions)
            ]
            
            # Constant velocity approximation
            velocities = [
                (end_pos - start_pos) / duration 
                for start_pos, end_pos in zip(start_positions, end_positions)
            ]
            
            # Create trajectory point
            point = JointTrajectoryPoint()
            point.positions = positions
            point.velocities = velocities
            point.accelerations = [0.0] * len(positions)
            point.time_from_start = Duration(sec=int(time_step * i), nanosec=int((time_step * i - int(time_step * i)) * 1e9))
            
            points.append(point)
        
        return points
    
    @staticmethod
    def generate_circular_trajectory(joint_names: List[str], center: List[float], radius: float, duration: float = 10.0, steps: int = 200) -> List[JointTrajectoryPoint]:
        """Generate circular trajectory (simplified for 2D motion)"""
        points = []
        time_step = duration / steps
        
        if len(joint_names) < 2:
            raise ValueError("Need at least 2 joints for circular trajectory")
        
        for i in range(steps + 1):
            t = i * 2 * np.pi / steps
            
            positions = center.copy()
            positions[0] += radius * np.cos(t)  # X coordinate
            positions[1] += radius * np.sin(t)  # Y coordinate
            
            # Velocities (tangential to circle)
            velocities = [0.0] * len(positions)
            velocities[0] = -radius * np.sin(t) * 2 * np.pi / duration
            velocities[1] = radius * np.cos(t) * 2 * np.pi / duration
            
            # Create trajectory point
            point = JointTrajectoryPoint()
            point.positions = positions
            point.velocities = velocities
            point.accelerations = [
                -radius * np.cos(t) * (2 * np.pi / duration)**2,
                -radius * np.sin(t) * (2 * np.pi / duration)**2
            ] + [0.0] * (len(positions) - 2)
            
            point.time_from_start = Duration(sec=int(time_step * i), nanosec=int((time_step * i - int(time_step * i)) * 1e9))
            
            points.append(point)
        
        return points

class ActionClientManager(Node):
    """Manager class to handle multiple action clients"""
    
    def __init__(self):
        super().__init__('action_client_manager')
        
        # Create multiple action clients
        self.traj_client = ActionClient(
            self,
            FollowJointTrajectory,
            'follow_joint_trajectory'
        )
        
        # Wait for servers
        self.get_logger().info('Waiting for action servers...')
        self.traj_client.wait_for_server()
        self.get_logger().info('Action servers available')
        
        # Create publishers for visualization
        self.progress_pub = self.create_publisher(Float64, 'action_progress', 10)
        
        # Timer for sending different trajectories
        self.trajectory_timer = self.create_timer(15.0, self.send_next_trajectory)
        self.trajectory_counter = 0
        self.trajectory_types = ['sinusoidal', 'linear', 'circular']
        
        self.get_logger().info('Action client manager initialized')
    
    def send_next_trajectory(self):
        """Send the next trajectory in sequence"""
        trajectory_type = self.trajectory_types[self.trajectory_counter % len(self.trajectory_types)]
        self.get_logger().info(f'Sending {trajectory_type} trajectory')
        
        if trajectory_type == 'sinusoidal':
            self.send_sinusoidal_trajectory()
        elif trajectory_type == 'linear':
            self.send_linear_trajectory()
        elif trajectory_type == 'circular':
            self.send_circular_trajectory()
        
        self.trajectory_counter += 1
    
    def send_sinusoidal_trajectory(self):
        """Send sinusoidal trajectory"""
        joint_names = ['joint1', 'joint2', 'joint3']
        trajectory = TrajectoryGenerator.generate_sinusoidal_trajectory(
            joint_names, duration=8.0, steps=100
        )
        
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = JointTrajectory()
        goal_msg.trajectory.joint_names = joint_names
        goal_msg.trajectory.points = trajectory
        
        # Send goal
        send_goal_future = self.traj_client.send_goal_async(
            goal_msg,
            feedback_callback=self.traj_feedback_callback
        )
        
        send_goal_future.add_done_callback(
            lambda future: self.traj_goal_response_callback(future, 'sinusoidal')
        )
    
    def send_linear_trajectory(self):
        """Send linear trajectory"""
        joint_names = ['joint1', 'joint2', 'joint3']
        start_positions = [0.0, 0.0, 0.0]
        end_positions = [1.5, -0.5, 2.0]
        
        trajectory = TrajectoryGenerator.generate_linear_trajectory(
            joint_names, start_positions, end_positions, duration=6.0, steps=80
        )
        
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = JointTrajectory()
        goal_msg.trajectory.joint_names = joint_names
        goal_msg.trajectory.points = trajectory
        
        # Send goal
        send_goal_future = self.traj_client.send_goal_async(
            goal_msg,
            feedback_callback=self.traj_feedback_callback
        )
        
        send_goal_future.add_done_callback(
            lambda future: self.traj_goal_response_callback(future, 'linear')
        )
    
    def send_circular_trajectory(self):
        """Send circular trajectory"""
        joint_names = ['joint1', 'joint2', 'joint3']
        
        try:
            trajectory = TrajectoryGenerator.generate_circular_trajectory(
                joint_names, center=[0.0, 0.0, 1.0], radius=0.5, duration=12.0, steps=200
            )
            
            goal_msg = FollowJointTrajectory.Goal()
            goal_msg.trajectory = JointTrajectory()
            goal_msg.trajectory.joint_names = joint_names[:2]  # Only first 2 joints for circular motion
            goal_msg.trajectory.points = [
                JointTrajectoryPoint(
                    positions=point.positions[:2],  # Take only first 2 positions
                    velocities=point.velocities[:2],
                    accelerations=point.accelerations[:2],
                    time_from_start=point.time_from_start
                ) for point in trajectory
            ]
            
            # Send goal
            send_goal_future = self.traj_client.send_goal_async(
                goal_msg,
                feedback_callback=self.traj_feedback_callback
            )
            
            send_goal_future.add_done_callback(
                lambda future: self.traj_goal_response_callback(future, 'circular')
            )
            
        except ValueError as e:
            self.get_logger().error(f'Error generating circular trajectory: {e}')
    
    def traj_feedback_callback(self, feedback_msg):
        """Handle trajectory feedback"""
        # Calculate progress percentage
        if feedback_msg.feedback:
            # Simplified progress calculation
            progress_msg = Float64()
            progress_msg.data = 50.0  # Simplified
            self.progress_pub.publish(progress_msg)
    
    def traj_goal_response_callback(self, future, trajectory_type):
        """Handle trajectory goal response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info(f'{trajectory_type.capitalize()} trajectory goal rejected')
            return
        
        self.get_logger().info(f'{trajectory_type.capitalize()} trajectory goal accepted')
        
        # Get result
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(
            lambda future: self.traj_result_callback(future, trajectory_type)
        )
    
    def traj_result_callback(self, future, trajectory_type):
        """Handle trajectory result"""
        result = future.result().result
        self.get_logger().info(f'{trajectory_type.capitalize()} trajectory completed: {result}')

def run_action_examples():
    """Run action examples"""
    rclpy.init()
    
    # Create action client manager
    manager = ActionClientManager()
    
    try:
        rclpy.spin(manager)
    except KeyboardInterrupt:
        pass
    finally:
        manager.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    run_action_examples()
```

## Real-World Examples

### Autonomous Navigation System

In an autonomous navigation system, services and actions play critical roles:
- **Services**: Map loading, global path planning, system diagnostics
- **Actions**: Local navigation, obstacle avoidance, route following

The navigation stack typically uses actions for MoveBase-like behaviors, where robots need to navigate to waypoints while providing continuous feedback about progress and potential issues.

### Robotic Manipulation

For robotic manipulation:
- **Services**: Gripper control, tool changes, safety system activation
- **Actions**: Pick and place operations, complex manipulation sequences

Manipulation actions often include feedback about grasp success, trajectory execution progress, and potential collisions.

### Multi-Robot Coordination

In multi-robot systems:
- **Services**: Resource allocation, task assignment, system state queries
- **Actions**: Coordinated movements, formation control, collaborative tasks

## Code Snippet Ideas: Advanced Communication Patterns

### 1. Service with Custom Message Types

First, create a custom service message:

```python
# robot_control_interfaces/srv/CoordinateTransform.srv
# Define custom service for coordinate transformation
geometry_msgs/Point source_point
string source_frame
string target_frame
---
geometry_msgs/Point transformed_point
bool success
string error_message
```

```cpp
// Implementation in C++
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/point.hpp"
#include "robot_control_interfaces/srv/coordinate_transform.hpp"

class CoordinateTransformService : public rclcpp::Node
{
public:
    CoordinateTransformService() : Node("coordinate_transform_service")
    {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        transform_service_ = this->create_service<robot_control_interfaces::srv::CoordinateTransform>(
            "transform_coordinates",
            std::bind(&CoordinateTransformService::handle_transform, this,
                     std::placeholders::_1, std::placeholders::_2)
        );
    }

private:
    void handle_transform(
        const robot_control_interfaces::srv::CoordinateTransform::Request::SharedPtr request,
        robot_control_interfaces::srv::CoordinateTransform::Response::SharedPtr response)
    {
        try {
            geometry_msgs::msg::TransformStamped transform = 
                tf_buffer_->lookupTransform(
                    request->target_frame, request->source_frame,
                    tf2::TimePointZero  // Latest available transform
                );
            
            // Apply transformation
            tf2::doTransform(request->source_point, response->transformed_point, transform);
            response->success = true;
            
        } catch (const tf2::TransformException & ex) {
            response->success = false;
            response->error_message = ex.what();
            RCLCPP_ERROR(this->get_logger(), "Transform failed: %s", ex.what());
        }
    }
    
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Service<robot_control_interfaces::srv::CoordinateTransform>::SharedPtr transform_service_;
};
```

### 2. Action with Multiple Goal Types

```cpp
// Multi-purpose action server
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

// Define custom action with multiple operation types
// navigation_action.msg
// # Operation type enum
// uint8 MOVE_TO_POSE = 1
// uint8 FOLLOW_PATH = 2
// uint8 GO_HOME = 3
//
// # Goal
// uint8 operation_type
// geometry_msgs/PoseStamped target_pose
// nav_msgs/Path path
// ---
// # Result
// bool success
// string message
// ---
// # Feedback
// float32 progress
// geometry_msgs/Pose current_pose

class MultipurposeNavigationServer : public rclcpp::Node
{
public:
    using NavigationAction = navigation_interfaces::action::NavigationAction;
    using GoalHandleNavigation = rclcpp_action::ServerGoalHandle<NavigationAction>;

    explicit MultipurposeNavigationServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("multipurpose_navigation_server", options)
    {
        action_server_ = rclcpp_action::create_server<NavigationAction>(
            this->get_node_base_interface(),
            this->get_node_clock_interface(),
            this->get_node_logging_interface(),
            this->get_node_waitables_interface(),
            "navigation_action",
            std::bind(&MultipurposeNavigationServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&MultipurposeNavigationServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&MultipurposeNavigationServer::handle_accepted, this, std::placeholders::_1)
        );
    }

private:
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const NavigationAction::Goal> goal)
    {
        // Validate operation type
        if (goal->operation_type != NavigationAction::Goal::MOVE_TO_POSE &&
            goal->operation_type != NavigationAction::Goal::FOLLOW_PATH &&
            goal->operation_type != NavigationAction::Goal::GO_HOME) {
            RCLCPP_WARN(this->get_logger(), "Invalid operation type: %d", goal->operation_type);
            return rclcpp_action::GoalResponse::REJECT;
        }
        
        // Check if robot is busy
        if (executing_operation_) {
            RCLCPP_WARN(this->get_logger(), "Operation rejected - robot busy");
            return rclcpp_action::GoalResponse::REJECT;
        }
        
        RCLCPP_INFO(this->get_logger(), "Accepting navigation goal of type %d", goal->operation_type);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    void execute(const std::shared_ptr<GoalHandleNavigation> goal_handle)
    {
        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<NavigationAction::Result>();
        
        executing_operation_ = true;
        
        switch (goal->operation_type) {
            case NavigationAction::Goal::MOVE_TO_POSE:
                result = execute_move_to_pose(goal_handle);
                break;
            case NavigationAction::Goal::FOLLOW_PATH:
                result = execute_follow_path(goal_handle);
                break;
            case NavigationAction::Goal::GO_HOME:
                result = execute_go_home(goal_handle);
                break;
            default:
                result->success = false;
                result->message = "Unknown operation type";
                goal_handle->succeed(result);
                break;
        }
        
        executing_operation_ = false;
    }

    std::shared_ptr<NavigationAction::Result> execute_move_to_pose(
        const std::shared_ptr<GoalHandleNavigation> goal_handle)
    {
        // Implementation for moving to a pose
        // This would contain the actual navigation logic
        auto result = std::make_shared<NavigationAction::Result>();
        result->success = true;
        result->message = "Move to pose completed";
        goal_handle->succeed(result);
        return result;
    }

    std::shared_ptr<NavigationAction::Result> execute_follow_path(
        const std::shared_ptr<GoalHandleNavigation> goal_handle)
    {
        // Implementation for following a path
        auto result = std::make_shared<NavigationAction::Result>();
        result->success = true;
        result->message = "Path following completed";
        goal_handle->succeed(result);
        return result;
    }

    std::shared_ptr<NavigationAction::Result> execute_go_home(
        const std::shared_ptr<GoalHandleNavigation> goal_handle)
    {
        // Implementation for returning home
        auto result = std::make_shared<NavigationAction::Result>();
        result->success = true;
        result->message = "Returned home successfully";
        goal_handle->succeed(result);
        return result;
    }

    rclcpp_action::Server<NavigationAction>::SharedPtr action_server_;
    std::atomic<bool> executing_operation_{false};
};
```

### 3. Service Mesh Pattern

```python
# Service mesh implementation for resilient communication
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from example_interfaces.srv import Trigger, SetBool
from std_msgs.msg import String
import asyncio
from concurrent.futures import ThreadPoolExecutor
import time
from typing import Dict, List, Optional, Callable
import threading

class ServiceMeshNode(Node):
    """Node that provides service mesh functionality"""
    
    def __init__(self):
        super().__init__('service_mesh_node')
        
        # Create service registry
        self.service_registry = {}
        self.service_health = {}
        self.service_stats = {}
        
        # Create health check service
        self.health_service = self.create_service(
            Trigger, 'mesh_health_check', self.health_check_callback
        )
        
        # Create service discovery service
        self.discovery_service = self.create_service(
            Trigger, 'service_discovery', self.service_discovery_callback
        )
        
        # Health monitoring timer
        self.health_timer = self.create_timer(5.0, self.monitor_services)
        
        self.get_logger().info('Service mesh node initialized')
    
    def register_service(self, service_name: str, service_type: str, node_name: str, address: str):
        """Register a service in the mesh"""
        if service_name not in self.service_registry:
            self.service_registry[service_name] = []
        
        service_info = {
            'service_type': service_type,
            'node_name': node_name,
            'address': address,
            'registered_time': time.time(),
            'last_heartbeat': time.time(),
            'status': 'HEALTHY'
        }
        
        self.service_registry[service_name].append(service_info)
        self.service_health[service_name] = {}
        self.service_stats[service_name] = {
            'requests': 0,
            'successes': 0,
            'failures': 0,
            'avg_response_time': 0.0
        }
        
        self.get_logger().info(f'Registered service: {service_name}')
    
    def health_check_callback(self, request, response):
        """Health check for the mesh itself"""
        active_services = sum(1 for service_list in self.service_registry.values() 
                             for service in service_list if service['status'] == 'HEALTHY')
        
        response.success = True
        response.message = f'Service mesh healthy - {active_services} active services'
        
        return response
    
    def service_discovery_callback(self, request, response):
        """Provide service discovery functionality"""
        services_info = {}
        for service_name, service_list in self.service_registry.items():
            services_info[service_name] = [
                {k: v for k, v in service.items() if k != 'status'} 
                for service in service_list
            ]
        
        response.success = True
        response.message = str(services_info)
        
        return response
    
    def monitor_services(self):
        """Monitor service health"""
        current_time = time.time()
        
        for service_name, service_list in self.service_registry.items():
            for service in service_list:
                # Check if heartbeat is stale
                if current_time - service['last_heartbeat'] > 10:  # 10 second timeout
                    service['status'] = 'UNHEALTHY'
                    self.get_logger().warning(f'Service {service_name} at {service["node_name"]} is unhealthy')
                
                # Update stats periodically
                if service_name in self.service_stats:
                    stats = self.service_stats[service_name]
                    if stats['requests'] > 0:
                        self.get_logger().debug(
                            f'Service {service_name} - '
                            f'Requests: {stats["requests"]}, '
                            f'Success rate: {(stats["successes"]/stats["requests"])*100:.1f}%'
                        )

class ServiceMeshClient(Node):
    """Client that uses service mesh for resilient communication"""
    
    def __init__(self):
        super().__init__('service_mesh_client')
        
        # Service mesh client
        self.mesh_health_client = self.create_client(Trigger, 'mesh_health_check')
        self.service_discovery_client = self.create_client(Trigger, 'service_discovery')
        
        # Wait for mesh services
        self.get_logger().info('Waiting for service mesh...')
        while not self.mesh_health_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service mesh not available, waiting...')
        
        self.get_logger().info('Service mesh available')
        
        # Load initial service registry
        self.load_service_registry()
        
        # Call services through mesh
        self.call_services_through_mesh()
    
    def load_service_registry(self):
        """Load initial service registry from discovery service"""
        request = Trigger.Request()
        future = self.service_discovery_client.call_async(request)
        
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        
        if response and response.success:
            self.get_logger().info(f'Service registry loaded: {response.message}')
    
    def call_services_through_mesh(self):
        """Call services using the mesh for resilience"""
        # Example: Call multiple services with retry logic
        services_to_call = [
            {'service_name': 'health_check', 'service_type': 'Trigger'},
            {'service_name': 'calibration', 'service_type': 'SetBool'}
        ]
        
        for service_info in services_to_call:
            self.call_with_retry(service_info)
    
    def call_with_retry(self, service_info: Dict, max_retries: int = 3, timeout: float = 5.0):
        """Call service with retry and timeout logic"""
        service_name = service_info['service_name']
        
        for attempt in range(max_retries):
            try:
                # Discover available instances
                request = Trigger.Request()
                future = self.service_discovery_client.call_async(request)
                
                rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
                
                if not future.done():
                    self.get_logger().warning(f'Service discovery timed out for {service_name}')
                    continue
                
                response = future.result()
                if response and response.success:
                    # In a real implementation, we would parse the service list
                    # and call the appropriate service instance
                    self.get_logger().info(f'Calling service {service_name}, attempt {attempt + 1}')
                    
                    # For demonstration, call a simple service
                    self.call_simple_service(service_name)
                    return  # Success
                
            except Exception as e:
                self.get_logger().error(f'Error calling {service_name}: {str(e)}')
                if attempt == max_retries - 1:
                    self.get_logger().error(f'All attempts failed for {service_name}')
    
    def call_simple_service(self, service_name: str):
        """Call a simple service for demonstration"""
        # In a real implementation, this would dynamically create clients
        # based on discovered service information
        self.get_logger().info(f'Simulated call to {service_name}')

def run_service_mesh_demo():
    """Run service mesh demonstration"""
    rclpy.init()
    
    mesh_node = ServiceMeshNode()
    mesh_client = ServiceMeshClient()
    
    # Create executor
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(mesh_node)
    executor.add_node(mesh_client)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        mesh_node.destroy_node()
        mesh_client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    run_service_mesh_demo()
```

## Best Practices and Patterns

### 1. Service Design Best Practices

1. **Keep Service Operations Idempotent**: When possible, design services so that calling them multiple times has the same effect as calling them once.

2. **Use Appropriate Service Types**: Choose between different service types based on operation characteristics:
   - Use `Trigger` for simple start/stop operations
   - Use `SetBool` for on/off toggles
   - Use custom service types for complex operations

3. **Implement Proper Error Handling**: Always handle exceptions and provide meaningful error messages.

4. **Consider Service Composition**: Combine multiple services to create higher-level functionality.

### 2. Action Best Practices

1. **Provide Meaningful Feedback**: Send feedback messages regularly during long operations.

2. **Handle Goal Preemption Gracefully**: Clean up resources properly when goals are canceled.

3. **Use Appropriate State Machines**: Implement state machines for complex action flows.

4. **Implement Timeout Logic**: Avoid indefinite waits by implementing timeouts.

## Integration with Other Patterns

### Combining Services and Actions

Many robotic applications benefit from combining different communication patterns:

```cpp
// Example: Navigation system that uses both services and actions
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

class IntegratedNavigationSystem : public rclcpp::Node
{
public:
    explicit IntegratedNavigationSystem(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("integrated_navigation_system", options)
    {
        // Action server for navigation
        nav_action_server_ = rclcpp_action::create_server<nav2_msgs::action::NavigateToPose>(
            this->get_node_base_interface(),
            this->get_node_clock_interface(),
            this->get_node_logging_interface(),
            this->get_node_waitables_interface(),
            "navigate_to_pose",
            std::bind(&IntegratedNavigationSystem::handle_nav_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&IntegratedNavigationSystem::handle_nav_cancel, this, std::placeholders::_1),
            std::bind(&IntegratedNavigationSystem::handle_nav_accepted, this, std::placeholders::_1)
        );
        
        // Service for immediate navigation commands
        quick_nav_service_ = this->create_service<std_srvs::srv::SetBool>(
            "quick_navigation",
            std::bind(&IntegratedNavigationSystem::handle_quick_nav, this,
                     std::placeholders::_1, std::placeholders::_2)
        );
        
        RCLCPP_INFO(this->get_logger(), "Integrated navigation system initialized");
    }

private:
    rclcpp_action::Server<nav2_msgs::action::NavigateToPose>::SharedPtr nav_action_server_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr quick_nav_service_;
    
    rclcpp_action::GoalResponse handle_nav_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const nav2_msgs::action::NavigateToPose::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Handling navigation goal");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    
    rclcpp_action::CancelResponse handle_nav_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::NavigateToPose>> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Handling navigation cancel");
        return rclcpp_action::CancelResponse::ACCEPT;
    }
    
    void handle_nav_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::NavigateToPose>> goal_handle)
    {
        std::thread{std::bind(&IntegratedNavigationSystem::execute_navigation, this, std::placeholders::_1), goal_handle}.detach();
    }
    
    void execute_navigation(const std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::NavigateToPose>> goal_handle)
    {
        // Navigation execution logic
        auto result = std::make_shared<nav2_msgs::action::NavigateToPose::Result>();
        result->result.poses.push_back(goal_handle->get_goal()->pose);
        
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Navigation completed");
    }
    
    void handle_quick_nav(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        if (request->data) {
            // Execute quick navigation (simplified)
            response->success = true;
            response->message = "Quick navigation executed";
            RCLCPP_INFO(this->get_logger(), "Quick navigation executed");
        } else {
            response->success = true;
            response->message = "Quick navigation aborted";
            RCLCPP_INFO(this->get_logger(), "Quick navigation aborted");
        }
    }
};
```

## Conclusion

Services and actions are fundamental communication patterns that extend beyond the basic publish-subscribe model, enabling more sophisticated robotic applications. Services provide synchronous request-reply communication suitable for immediate operations, while actions enable long-running tasks with feedback and preemption capabilities.

The implementation of these patterns requires careful consideration of system requirements, error handling, and user expectations. With proper design, services and actions can provide robust, responsive robotic systems capable of handling complex behaviors and interactions.

Understanding these patterns is essential for developing scalable robotic applications that can grow from simple research projects to complex production systems. The Quality of Service policies and advanced features of ROS 2 further enhance these patterns, making them suitable for mission-critical applications requiring reliability and predictability.

In the following chapters, we'll explore advanced topics including parameter management, launch files, and complex system architectures that build upon these communication patterns.