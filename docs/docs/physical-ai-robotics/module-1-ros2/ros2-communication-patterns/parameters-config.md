---
sidebar_position: 5
sidebar_label: Advanced Parameters and Launch Configuration
---

# Advanced Parameters and Launch Configuration in ROS 2

## Introduction to Advanced Parameter Management

The parameter system in ROS 2 represents a significant advancement over ROS 1, providing enhanced type safety, dynamic reconfiguration, and sophisticated management capabilities. Parameters serve as the primary mechanism for configuring node behavior without requiring recompilation, enabling runtime adaptation, and facilitating the deployment of generic nodes across different robotic platforms.

ROS 2 parameters go beyond simple key-value storage by providing typed values, validation mechanisms, dynamic reconfiguration, and structured hierarchical organization. This system enables complex robotic applications to adapt their behavior based on configuration while maintaining type consistency and runtime safety.

The parameter system in ROS 2 is built around several key concepts:
- **Typed Parameters**: Each parameter has a specific type that is enforced at runtime
- **Parameter Descriptors**: Metadata that describes valid ranges, values, and relationships
- **Dynamic Reconfiguration**: Parameters can be modified during runtime with validation
- **Hierarchical Namespaces**: Structured parameter organization that supports composition and reuse
- **Parameter Files**: Persistent parameter storage with multiple format support (YAML, JSON)

The design philosophy emphasizes configuration flexibility while maintaining system stability and safety. Parameters enable the same node to operate differently in simulation versus reality, or to adapt to different hardware configurations without requiring recompilation.

## Deep Technical Analysis of Parameter Systems

### Parameter Declaration and Type System

ROS 2 provides a sophisticated parameter declaration mechanism that enables early validation and clear interfaces. Parameters can be declared with specific types, default values, validators, and metadata.

```cpp
#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/integer_range.hpp"
#include "rcl_interfaces/msg/floating_point_range.hpp"
#include <limits>
#include <regex>

class AdvancedParameterNode : public rclcpp::Node
{
public:
    explicit AdvancedParameterNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("advanced_parameter_node", options)
    {
        // Initialize parameter change callback
        this->register_param_change_callback(
            std::bind(&AdvancedParameterNode::on_parameter_change, this, std::placeholders::_1)
        );
        
        // Declare parameters with full descriptors
        declare_motion_parameters();
        declare_sensor_parameters();
        declare_communication_parameters();
        declare_performance_parameters();
        
        // Setup parameter validation timer
        param_validation_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&AdvancedParameterNode::validate_parameters, this)
        );
        
        RCLCPP_INFO(this->get_logger(), "Advanced parameter node initialized with %zu parameters", 
                   this->list_parameters({}).names.size());
    }

private:
    void declare_motion_parameters()
    {
        // Motion control parameters with validation
        rcl_interfaces::msg::ParameterDescriptor motion_desc;
        motion_desc.description = "Robot motion control parameters";
        motion_desc.read_only = false;
        motion_desc.dynamic_typing = false;
        
        // Linear velocity constraints
        rcl_interfaces::msg::ParameterDescriptor linear_desc;
        linear_desc.description = "Maximum linear velocity (m/s)";
        linear_desc.read_only = false;
        
        // Add floating point range
        rcl_interfaces::msg::FloatingPointRange linear_range;
        linear_range.from_value = 0.0;
        linear_range.to_value = 5.0;
        linear_range.step = 0.01;
        linear_desc.floating_point_range = {linear_range};
        
        this->declare_parameter("motion.max_linear_velocity", 1.0, linear_desc);
        
        // Angular velocity constraints
        rcl_interfaces::msg::ParameterDescriptor angular_desc;
        angular_desc.description = "Maximum angular velocity (rad/s)";
        angular_desc.read_only = false;
        
        rcl_interfaces::msg::FloatingPointRange angular_range;
        angular_range.from_value = 0.0;
        angular_range.to_value = 3.14159;
        angular_range.step = 0.001;
        angular_desc.floating_point_range = {angular_range};
        
        this->declare_parameter("motion.max_angular_velocity", 1.57, angular_desc);
        
        // Acceleration constraints
        rcl_interfaces::msg::ParameterDescriptor acc_desc;
        acc_desc.description = "Robot acceleration limits (m/s²)";
        acc_desc.read_only = false;
        
        rcl_interfaces::msg::FloatingPointRange acc_range;
        acc_range.from_value = 0.1;
        acc_range.to_value = 10.0;
        acc_range.step = 0.1;
        acc_desc.floating_point_range = {acc_range};
        
        this->declare_parameter("motion.linear_acceleration", 2.0, acc_desc);
        this->declare_parameter("motion.angular_acceleration", 3.14, acc_desc);
    }
    
    void declare_sensor_parameters()
    {
        // Sensor configuration parameters
        rcl_interfaces::msg::ParameterDescriptor sensor_desc;
        sensor_desc.description = "Sensor configuration parameters";
        
        // Laser scanner resolution
        rcl_interfaces::msg::ParameterDescriptor resolution_desc;
        resolution_desc.description = "Laser scanner angular resolution (degrees)";
        resolution_desc.read_only = false;
        
        rcl_interfaces::msg::IntegerRange res_range;
        res_range.from_value = 1;
        res_range.to_value = 5;
        res_range.step = 1;
        resolution_desc.integer_range = {res_range};
        
        this->declare_parameter("sensor.laser_resolution", 1, resolution_desc);
        
        // Camera parameters
        this->declare_parameter("sensor.camera_width", 640);
        this->declare_parameter("sensor.camera_height", 480);
        this->declare_parameter("sensor.camera_fps", 30);
        
        // Sensor fusion weights
        std::vector<rclcpp::Parameter> fusion_weights = {
            rclcpp::Parameter("sensor.weights.lidar", 0.6),
            rclcpp::Parameter("sensor.weights.camera", 0.3),
            rclcpp::Parameter("sensor.weights.imu", 0.1)
        };
        
        for (const auto& param : fusion_weights) {
            this->declare_parameter(param.get_name(), param.as_double());
        }
    }
    
    void declare_communication_parameters()
    {
        // Communication and networking parameters
        rcl_interfaces::msg::ParameterDescriptor comm_desc;
        comm_desc.description = "Communication and networking parameters";
        
        // Quality of Service settings
        this->declare_parameter("qos.reliability", "reliable");
        this->declare_parameter("qos.durability", "volatile");
        this->declare_parameter("qos.history_depth", 10);
        
        // Network bandwidth limits
        rcl_interfaces::msg::ParameterDescriptor bandwidth_desc;
        bandwidth_desc.description = "Network bandwidth limits (Mbps)";
        rcl_interfaces::msg::IntegerRange bw_range;
        bw_range.from_value = 1;
        bw_range.to_value = 1000;
        bw_range.step = 1;
        bandwidth_desc.integer_range = {bw_range};
        
        this->declare_parameter("network.max_bandwidth", 100, bandwidth_desc);
        this->declare_parameter("network.buffer_size", 1024);
    }
    
    void declare_performance_parameters()
    {
        // Performance and timing parameters
        rcl_interfaces::msg::ParameterDescriptor perf_desc;
        perf_desc.description = "Performance and timing parameters";
        
        // Real-time execution parameters
        this->declare_parameter("timing.control_rate", 100.0);  // Hz
        this->declare_parameter("timing.max_execution_time", 0.01);  // s
        
        // Resource usage limits
        rcl_interfaces::msg::ParameterDescriptor resource_desc;
        resource_desc.description = "Resource usage limits (%)";
        rcl_interfaces::msg::FloatingPointRange resource_range;
        resource_range.from_value = 1.0;
        resource_range.to_value = 100.0;
        resource_range.step = 0.1;
        resource_desc.floating_point_range = {resource_range};
        
        this->declare_parameter("resources.cpu_limit", 80.0, resource_desc);
        this->declare_parameter("resources.memory_limit", 80.0, resource_desc);
    }
    
    rcl_interfaces::msg::SetParametersResult on_parameter_change(const std::vector<rclcpp::Parameter> & parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "Parameters set successfully";
        
        for (const auto & param : parameters) {
            std::string name = param.get_name();
            rclcpp::ParameterType type = param.get_type();
            
            RCLCPP_DEBUG(this->get_logger(), "Parameter change: %s = %s", 
                        name.c_str(), param.value_to_string().c_str());
            
            // Perform parameter-specific validation
            if (name == "motion.max_linear_velocity") {
                double value = param.as_double();
                if (value < 0.0 || value > 10.0) {
                    result.successful = false;
                    result.reason = "Linear velocity must be between 0.0 and 10.0 m/s";
                    return result;
                }
            }
            else if (name == "motion.max_angular_velocity") {
                double value = param.as_double();
                if (value < 0.0 || value > 6.28) {
                    result.successful = false;
                    result.reason = "Angular velocity must be between 0.0 and 6.28 rad/s";
                    return result;
                }
            }
            else if (name == "sensor.laser_resolution") {
                int value = param.as_int();
                if (value < 1 || value > 5) {
                    result.successful = false;
                    result.reason = "Laser resolution must be between 1 and 5 degrees";
                    return result;
                }
            }
        }
        
        // Apply parameter changes to system
        apply_parameter_changes(parameters);
        
        return result;
    }
    
    void apply_parameter_changes(const std::vector<rclcpp::Parameter> & parameters)
    {
        // Apply validated parameter changes to the running system
        for (const auto & param : parameters) {
            std::string name = param.get_name();
            
            if (name == "motion.max_linear_velocity") {
                current_max_linear_vel_ = param.as_double();
                RCLCPP_INFO(this->get_logger(), "Max linear velocity updated to: %.2f", current_max_linear_vel_);
            }
            else if (name == "motion.max_angular_velocity") {
                current_max_angular_vel_ = param.as_double();
                RCLCPP_INFO(this->get_logger(), "Max angular velocity updated to: %.2f", current_max_angular_vel_);
            }
            // Add more parameter applications as needed
        }
        
        // Notify other subsystems of parameter changes
        notify_parameter_updates();
    }
    
    void validate_parameters()
    {
        // Periodic validation of parameter constraints
        double linear_vel = this->get_parameter("motion.max_linear_velocity").as_double();
        double angular_vel = this->get_parameter("motion.max_angular_velocity").as_double();
        int resolution = this->get_parameter("sensor.laser_resolution").as_int();
        
        bool valid = true;
        std::string validation_msg;
        
        if (linear_vel < 0.0 || linear_vel > 10.0) {
            valid = false;
            validation_msg += "Linear velocity out of bounds; ";
        }
        
        if (angular_vel < 0.0 || angular_vel > 6.28) {
            valid = false;
            validation_msg += "Angular velocity out of bounds; ";
        }
        
        if (resolution < 1 || resolution > 5) {
            valid = false;
            validation_msg += "Laser resolution out of bounds; ";
        }
        
        if (!valid) {
            RCLCPP_WARN_ONCE(this->get_logger(), "Parameter validation failed: %s", validation_msg.c_str());
        }
    }
    
    void notify_parameter_updates()
    {
        // Notify other system components of parameter changes
        // This could involve publishing notification messages or calling callbacks
    }
    
    // Current parameter values for runtime access
    double current_max_linear_vel_ = 1.0;
    double current_max_angular_vel_ = 1.57;
    
    // Timer for periodic validation
    rclcpp::TimerBase::SharedPtr param_validation_timer_;
};
```

### Parameter File Management and Configuration

ROS 2 provides sophisticated mechanisms for managing parameters through external configuration files. This enables system configuration to be abstracted from implementation and supports different deployment environments.

```yaml
# Example parameter configuration file: robot_config.yaml
/**
  ros__parameters:
    # Motion control parameters
    motion:
      max_linear_velocity: 2.0
      max_angular_velocity: 3.14
      linear_acceleration: 3.0
      angular_acceleration: 5.0
    
    # Sensor configuration
    sensor:
      laser_resolution: 1
      camera_width: 1280
      camera_height: 720
      camera_fps: 60
      weights:
        lidar: 0.5
        camera: 0.3
        imu: 0.2
    
    # Communication settings
    qos:
      reliability: "reliable"
      durability: "transient_local"
      history_depth: 20
    
    # Performance parameters
    timing:
      control_rate: 200.0
      max_execution_time: 0.005
    
    # Resource limits
    resources:
      cpu_limit: 75.0
      memory_limit: 70.0
```

```python
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
import yaml
import json
from typing import Dict, Any, Optional, Union
import os
import tempfile
from pathlib import Path

class ParameterManagerNode(Node):
    def __init__(self):
        super().__init__('parameter_manager')
        
        # Initialize parameter management
        self.parameter_registry = {}
        self.parameter_descriptors = {}
        self.configuration_history = []
        self.validation_schemas = {}
        
        # Setup parameter management services
        self.setup_parameter_management()
        
        # Load configuration files
        self.load_configuration_files()
        
        # Setup configuration change monitoring
        self.config_monitor_timer = self.create_timer(2.0, self.monitor_configuration_changes)
        
        # Publisher for configuration notifications
        self.config_change_pub = self.create_publisher(String, 'configuration_changes', 10)
        
        self.get_logger().info('Parameter manager initialized')
    
    def setup_parameter_management(self):
        """Setup parameter management services and interfaces"""
        # Declare configuration parameters
        self.declare_parameter('config_file_path', '~/robot_configs/')
        self.declare_parameter('auto_reload_enabled', True)
        self.declare_parameter('validation_enabled', True)
        self.declare_parameter('backup_enabled', True)
        self.declare_parameter('change_notification_enabled', True)
        
        # Create parameter change callback
        self.add_on_set_parameters_callback(self.parameter_change_callback)
        
        # Setup parameter validation schemas
        self.setup_validation_schemas()
    
    def setup_validation_schemas(self):
        """Setup validation schemas for different parameter types"""
        # Motion validation schema
        self.validation_schemas['motion'] = {
            'max_linear_velocity': {
                'type': 'float',
                'min': 0.0,
                'max': 10.0,
                'required': True
            },
            'max_angular_velocity': {
                'type': 'float',
                'min': 0.0,
                'max': 6.28,
                'required': True
            }
        }
        
        # Sensor validation schema
        self.validation_schemas['sensor'] = {
            'laser_resolution': {
                'type': 'int',
                'min': 1,
                'max': 5,
                'required': True
            },
            'camera_width': {
                'type': 'int',
                'min': 160,
                'max': 1920,
                'required': False,
                'default': 640
            }
        }
        
        # QoS validation schema
        self.validation_schemas['qos'] = {
            'reliability': {
                'type': 'string',
                'allowed_values': ['reliable', 'best_effort', 'unreliable'],
                'required': True
            }
        }
    
    def parameter_change_callback(self, parameters):
        """Handle parameter change requests"""
        result = Parameter.Type.PARAMETER_NOT_SET
        
        for param in parameters:
            param_name = param.name
            param_value = param.value
            param_type = param.type_()
            
            # Validate parameter
            validation_result = self.validate_parameter(param_name, param_value, param_type)
            
            if validation_result['valid']:
                RCLCPP_DEBUG(self.get_logger(), f'Valid parameter change: {param_name} = {param_value}')
                result = Parameter.Type.PARAMETER_SET
            else:
                RCLCPP_ERROR(self.get_logger(), f'Invalid parameter change: {validation_result["reason"]}')
                result = Parameter.Type.PARAMETER_NOT_SET
                return result  # Reject the entire change if any parameter is invalid
        
        # Apply parameter changes
        for param in parameters:
            self.apply_parameter_change(param)
        
        # Notify about parameter changes
        self.notify_parameter_change(parameters)
        
        return result
    
    def validate_parameter(self, name: str, value: Any, param_type: Parameter.Type) -> Dict[str, Any]:
        """Validate parameter against schema"""
        result = {'valid': True, 'reason': ''}
        
        # Check parameter type
        expected_type = self.get_expected_type(name)
        if expected_type is not None and not self.check_type_match(param_type, expected_type):
            result['valid'] = False
            result['reason'] = f'Parameter {name} expects type {expected_type}, got {param_type}'
            return result
        
        # Validate against schema if available
        schema = self.get_schema_for_parameter(name)
        if schema:
            # Validate type
            if schema.get('type') == 'float' and not isinstance(value, (int, float)):
                result['valid'] = False
                result['reason'] = f'Parameter {name} expects float, got {type(value)}'
            
            elif schema.get('type') == 'int' and not isinstance(value, int):
                result['valid'] = False
                result['reason'] = f'Parameter {name} expects int, got {type(value)}'
            
            elif schema.get('type') == 'string' and not isinstance(value, str):
                result['valid'] = False
                result['reason'] = f'Parameter {name} expects string, got {type(value)}'
            
            # Validate range if specified
            if 'min' in schema and value < schema['min']:
                result['valid'] = False
                result['reason'] = f'Parameter {name} value {value} below minimum {schema["min"]}'
            
            if 'max' in schema and value > schema['max']:
                result['valid'] = False
                result['reason'] = f'Parameter {name} value {value} above maximum {schema["max"]}'
            
            # Validate allowed values if specified
            if 'allowed_values' in schema and value not in schema['allowed_values']:
                result['valid'] = False
                result['reason'] = f'Parameter {name} value {value} not in allowed values: {schema["allowed_values"]}'
        
        return result
    
    def get_expected_type(self, name: str) -> Optional[Parameter.Type]:
        """Get expected type for a parameter"""
        schema = self.get_schema_for_parameter(name)
        if schema and 'type' in schema:
            type_map = {
                'int': Parameter.Type.PARAMETER_INTEGER,
                'float': Parameter.Type.PARAMETER_DOUBLE,
                'string': Parameter.Type.PARAMETER_STRING,
                'bool': Parameter.Type.PARAMETER_BOOL
            }
            return type_map.get(schema['type'])
        return None
    
    def check_type_match(self, actual_type: Parameter.Type, expected_type: Parameter.Type) -> bool:
        """Check if parameter types match"""
        return actual_type == expected_type
    
    def get_schema_for_parameter(self, name: str) -> Optional[Dict[str, Any]]:
        """Get validation schema for parameter"""
        # Split parameter name into hierarchical parts
        parts = name.split('.')
        current_schema = self.validation_schemas
        
        for part in parts[:-1]:
            if part in current_schema:
                current_schema = current_schema[part].get('children', {})
            else:
                return None
        
        final_part = parts[-1]
        if final_part in current_schema:
            return current_schema[final_part]
        
        return None
    
    def apply_parameter_change(self, param):
        """Apply validated parameter change"""
        # Store parameter in registry
        self.parameter_registry[param.name] = {
            'value': param.value,
            'type': param.type_(),
            'timestamp': self.get_clock().now()
        }
        
        # Update node's parameter
        self.set_parameters([param])
        
        self.get_logger().debug(f'Applied parameter change: {param.name} = {param.value}')
    
    def notify_parameter_change(self, parameters):
        """Notify about parameter changes"""
        if self.get_parameter('change_notification_enabled').value:
            for param in parameters:
                notification_msg = String()
                notification_msg.data = f'PARAMETER_CHANGE: {param.name} = {param.value}'
                self.config_change_pub.publish(notification_msg)
                
                self.get_logger().info(f'Parameter change notified: {notification_msg.data}')
    
    def load_configuration_files(self):
        """Load parameter configuration files"""
        config_path = self.get_parameter('config_file_path').value
        self.config_path = Path(os.path.expanduser(config_path))
        
        if not self.config_path.exists():
            self.get_logger().info(f'Configuration path does not exist: {self.config_path}')
            return
        
        # Load all YAML configuration files
        for config_file in self.config_path.glob('*.yaml'):
            self.load_yaml_config(config_file)
        
        # Load all JSON configuration files
        for config_file in self.config_path.glob('*.json'):
            self.load_json_config(config_file)
    
    def load_yaml_config(self, config_file: Path):
        """Load YAML configuration file"""
        try:
            with open(config_file, 'r') as f:
                config_data = yaml.safe_load(f)
            
            if config_data:
                self.apply_configuration(config_data, str(config_file))
                self.get_logger().info(f'Loaded YAML configuration: {config_file}')
            
        except Exception as e:
            self.get_logger().error(f'Error loading YAML config {config_file}: {str(e)}')
    
    def load_json_config(self, config_file: Path):
        """Load JSON configuration file"""
        try:
            with open(config_file, 'r') as f:
                config_data = json.load(f)
            
            if config_data:
                self.apply_configuration(config_data, str(config_file))
                self.get_logger().info(f'Loaded JSON configuration: {config_file}')
            
        except Exception as e:
            self.get_logger().error(f'Error loading JSON config {config_file}: {str(e)}')
    
    def apply_configuration(self, config_data: Dict[str, Any], source: str):
        """Apply configuration from loaded data"""
        if 'ros__parameters' in config_data:
            self.apply_nested_parameters(config_data['ros__parameters'])
        else:
            self.apply_nested_parameters(config_data)
    
    def apply_nested_parameters(self, config_dict: Dict[str, Any], prefix: str = ""):
        """Recursively apply nested parameter configuration"""
        for key, value in config_dict.items():
            full_param_name = f"{prefix}.{key}" if prefix else key
            
            if isinstance(value, dict):
                # Recursively handle nested dictionaries
                self.apply_nested_parameters(value, full_param_name)
            else:
                # Set the parameter if it's not already declared
                if not self.has_parameter(full_param_name):
                    self.declare_parameter(full_param_name, value)
                else:
                    # Update existing parameter
                    param = Parameter(full_param_name, Parameter.Type.PARAMETER_NOT_SET, value)
                    self.set_parameters([param])
                
                self.get_logger().debug(f'Set parameter: {full_param_name} = {value}')
    
    def monitor_configuration_changes(self):
        """Monitor configuration files for changes and reload if needed"""
        if not self.get_parameter('auto_reload_enabled').value:
            return
        
        # Check if configuration files have been modified
        config_files = list(self.config_path.glob('*.yaml')) + list(self.config_path.glob('*.json'))
        
        for config_file in config_files:
            # For demonstration, we'll just log file modifications
            # In a real system, you'd compare timestamps and reload if necessary
            pass
    
    def save_configuration_snapshot(self) -> str:
        """Save current configuration to temporary file"""
        config_snapshot = {
            'timestamp': self.get_clock().now().to_msg().sec,
            'parameters': {}
        }
        
        # Get all parameters
        param_list = self.list_parameters([])
        for param_name in param_list.names:
            param_value = self.get_parameter(param_name).value
            config_snapshot['parameters'][param_name] = param_value
        
        # Create temporary file
        temp_dir = Path(tempfile.mkdtemp())
        snapshot_file = temp_dir / 'config_snapshot.yaml'
        
        with open(snapshot_file, 'w') as f:
            yaml.dump(config_snapshot, f, default_flow_style=False)
        
        self.get_logger().info(f'Configuration snapshot saved to: {snapshot_file}')
        return str(snapshot_file)
    
    def get_parameter_summary(self) -> Dict[str, Any]:
        """Get summary of current parameter state"""
        param_list = self.list_parameters([])
        
        summary = {
            'total_parameters': len(param_list.names),
            'parameter_names': param_list.names,
            'configuration_timestamp': self.get_clock().now().to_msg().sec,
            'parameter_counts_by_type': {},
            'parameter_hierarchy': {}
        }
        
        # Count parameters by type and build hierarchy
        for param_name in param_list.names:
            param_value = self.get_parameter(param_name).value
            param_type = type(param_value).__name__
            
            # Count by type
            if param_type not in summary['parameter_counts_by_type']:
                summary['parameter_counts_by_type'][param_type] = 0
            summary['parameter_counts_by_type'][param_type] += 1
        
        return summary
    
    def reset_to_defaults(self):
        """Reset all parameters to their default values"""
        # This would typically require knowing the default values
        # For now, log the operation
        self.get_logger().info('Reset to defaults requested - this would reset all parameters to defaults')

def create_parameter_descriptor(name: str, 
                              description: str, 
                              param_type: Parameter.Type,
                              min_value: Union[int, float] = None,
                              max_value: Union[int, float] = None,
                              allowed_values: list = None,
                              read_only: bool = False) -> rcl_interfaces.msg.ParameterDescriptor:
    """Create a parameter descriptor with validation"""
    descriptor = rcl_interfaces.msg.ParameterDescriptor()
    descriptor.name = name
    descriptor.description = description
    descriptor.read_only = read_only
    descriptor.dynamic_typing = False
    
    if param_type == Parameter.Type.PARAMETER_INTEGER and min_value is not None and max_value is not None:
        int_range = rcl_interfaces.msg.IntegerRange()
        int_range.from_value = min_value
        int_range.to_value = max_value
        int_range.step = 1
        descriptor.integer_range = [int_range]
    
    elif param_type == Parameter.Type.PARAMETER_DOUBLE and min_value is not None and max_value is not None:
        float_range = rcl_interfaces.msg.FloatingPointRange()
        float_range.from_value = float(min_value)
        float_range.to_value = float(max_value)
        float_range.step = 0.0  # Continuous range
        descriptor.floating_point_range = [float_range]
    
    if allowed_values:
        descriptor.additional_constraints = f"Allowed values: {allowed_values}"
    
    return descriptor

def main():
    rclpy.init()
    
    param_node = ParameterManagerNode()
    
    try:
        rclpy.spin(param_node)
    except KeyboardInterrupt:
        param_node.get_logger().info('Shutting down parameter manager')
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Advanced Launch Configuration

### Complex Launch File Structures

Launch files in ROS 2 provide sophisticated mechanisms for managing complex robotic systems, supporting conditional launches, parameter inheritance, and complex node dependencies. Advanced launch configurations enable deployment of sophisticated multi-node systems with proper initialization, shutdown, and error handling.

```python
# advanced_launch_manager.py
from launch import LaunchDescription, LaunchContext
from launch.actions import (
    DeclareLaunchArgument, 
    OpaqueFunction,
    GroupAction,
    PopEnvironment, 
    PushEnvironment,
    RegisterEventHandler,
    TimerAction
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.events import Shutdown
from launch.substitutions import (
    LaunchConfiguration,
    TextSubstitution,
    PathJoinSubstitution,
    EnvironmentVariable
)
from launch_ros.actions import Node, PushRosNamespace, SetParameter, SetRemap
from launch_ros.substitutions import FindPackageShare
from launch.utilities import normalize_to_list_of_substitutions
import launch
import launch_ros
import yaml
import os
from typing import Dict, List
import sys

def generate_launch_description():
    """Generate complex launch description with advanced features"""
    
    # Declare launch arguments
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='robot',
        description='Robot namespace for multi-robot deployment'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        choices=['true', 'false'],
        description='Use simulation time'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=[FindPackageShare('my_robot_package'), '/config/robot_config.yaml'],
        description='Path to configuration file'
    )
    
    debug_mode_arg = DeclareLaunchArgument(
        'debug_mode',
        default_value='false',
        choices=['true', 'false'],
        description='Enable debug mode'
    )
    
    # Conditional parameters
    enable_gui_arg = DeclareLaunchArgument(
        'enable_gui',
        default_value='false',
        choices=['true', 'false'],
        description='Enable graphical user interface'
    )
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        choices=['debug', 'info', 'warn', 'error', 'fatal'],
        description='Logging level'
    )
    
    # Complex launch configuration with multiple components
    def launch_setup(context: LaunchContext):
        """Setup complex launch configuration"""
        namespace = LaunchConfiguration('namespace').perform(context)
        use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
        debug_mode = LaunchConfiguration('debug_mode').perform(context)
        enable_gui = LaunchConfiguration('enable_gui').perform(context)
        log_level = LaunchConfiguration('log_level').perform(context)
        
        launch_actions = []
        
        # Setup ROS namespace
        with_namespace = PushRosNamespace(namespace)
        launch_actions.append(with_namespace)
        
        # Set global parameters
        launch_actions.extend([
            SetParameter('use_sim_time', use_sim_time),
            SetParameter('debug_mode', debug_mode)
        ])
        
        # Create robot core nodes group
        core_nodes = create_core_nodes(context)
        launch_actions.extend(core_nodes)
        
        # Create sensor processing nodes group
        sensor_nodes = create_sensor_processing_nodes(context)
        launch_actions.extend(sensor_nodes)
        
        # Create navigation stack nodes group
        nav_nodes = create_navigation_nodes(context)
        launch_actions.extend(nav_nodes)
        
        # Conditionally create GUI nodes
        if enable_gui.lower() == 'true':
            gui_nodes = create_gui_nodes(context)
            launch_actions.extend(gui_nodes)
        
        # Add monitoring and diagnostics
        monitoring_nodes = create_monitoring_nodes(context)
        launch_actions.extend(monitoring_nodes)
        
        # Create error handling and recovery procedures
        launch_actions.extend(create_error_handling_procedures(context))
        
        # Add logging configuration
        launch_actions.extend(create_logging_configuration(log_level))
        
        return launch_actions
    
    # Create the main launch description
    launch_description = LaunchDescription([
        namespace_arg,
        use_sim_time_arg,
        config_file_arg,
        debug_mode_arg,
        enable_gui_arg,
        log_level_arg,
        OpaqueFunction(function=launch_setup)
    ])
    
    return launch_description

def create_core_nodes(context: LaunchContext) -> List:
    """Create core robot nodes"""
    namespace = LaunchConfiguration('namespace').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    config_file = LaunchConfiguration('config_file').perform(context)
    log_level = LaunchConfiguration('log_level').perform(context)
    
    nodes = []
    
    # Robot driver node
    robot_driver = Node(
        package='my_robot_drivers',
        executable='robot_driver',
        name='robot_driver',
        namespace=namespace,
        parameters=[
            {'use_sim_time': use_sim_time.lower() == 'true'},
            {'config_file': config_file}
        ],
        remappings=[
            ('/cmd_vel', 'cmd_vel'),
            ('/odom', 'odom'),
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ],
        arguments=[f'--log-level={log_level}'],
        respawn=True,
        respawn_delay=2.0,
        on_exit=launch.actions.LogInfo(msg='Robot driver exited')
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        parameters=[
            {'use_sim_time': use_sim_time.lower() == 'true'},
            {'publish_frequency': 50.0}
        ],
        arguments=[f'--log-level={log_level}'],
        on_exit=launch.actions.LogInfo(msg='Robot state publisher exited')
    )
    
    # Joint state publisher (conditional)
    joint_state_publisher = Node(
        condition=UnlessCondition(LaunchConfiguration('debug_mode')),
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace=namespace,
        parameters=[
            {'use_sim_time': use_sim_time.lower() == 'true'}
        ],
        arguments=[f'--log-level={log_level}'],
        on_exit=launch.actions.LogInfo(msg='Joint state publisher exited')
    )
    
    nodes.extend([robot_driver, robot_state_publisher, joint_state_publisher])
    
    return nodes

def create_sensor_processing_nodes(context: LaunchContext) -> List:
    """Create sensor processing nodes"""
    namespace = LaunchConfiguration('namespace').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    log_level = LaunchConfiguration('log_level').perform(context)
    
    nodes = []
    
    # Laser scan processor
    laser_scan_processor = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='laser_scan_processor',
        namespace=namespace,
        parameters=[
            {'use_sim_time': use_sim_time.lower() == 'true'},
            {'scan_topic': 'scan'},
            {'output_frame': 'base_link'}
        ],
        arguments=[f'--log-level={log_level}'],
        on_exit=launch.actions.LogInfo(msg='Laser scan processor exited')
    )
    
    # Camera processing node
    camera_processor = Node(
        package='image_proc',
        executable='image_proc',
        name='camera_processor',
        namespace=namespace,
        parameters=[
            {'use_sim_time': use_sim_time.lower() == 'true'},
            {'rectified_publishing_enabled': True}
        ],
        remappings=[
            ('image_raw', 'camera/image_raw'),
            ('image_rect', 'camera/image_rect')
        ],
        arguments=[f'--log-level={log_level}'],
        on_exit=launch.actions.LogInfo(msg='Camera processor exited')
    )
    
    # IMU preprocessor
    imu_preprocessor = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_node',
        name='imu_preprocessor',
        namespace=namespace,
        parameters=[
            {'use_sim_time': use_sim_time.lower() == 'true'},
            {'fixed_frame': 'base_link'},
            {'publish_tf': False}
        ],
        remappings=[
            ('imu/data_raw', 'imu_raw'),
            ('imu/data', 'imu_filtered')
        ],
        arguments=[f'--log-level={log_level}'],
        on_exit=launch.actions.LogInfo(msg='IMU preprocessor exited')
    )
    
    nodes.extend([laser_scan_processor, camera_processor, imu_preprocessor])
    
    return nodes

def create_navigation_nodes(context: LaunchContext) -> List:
    """Create navigation stack nodes"""
    namespace = LaunchConfiguration('namespace').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    log_level = LaunchConfiguration('log_level').perform(context)
    
    nodes = []
    
    # Costmap server
    costmap_server = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        name='costmap_server',
        namespace=namespace,
        parameters=[
            {'use_sim_time': use_sim_time.lower() == 'true'},
            {'plugins': ['obstacle_layer', 'inflation_layer']},
            {'obstacle_layer.enabled': True},
            {'inflation_layer.enabled': True}
        ],
        arguments=[f'--log-level={log_level}'],
        on_exit=launch.actions.LogInfo(msg='Costmap server exited')
    )
    
    # Planner server
    planner_server = Node(
        package='nav2_planner',
        executable='nav2_planner',
        name='planner_server',
        namespace=namespace,
        parameters=[
            {'use_sim_time': use_sim_time.lower() == 'true'},
            {'planner_plugin': 'nav2_navfn_planner/NavfnPlanner'}
        ],
        arguments=[f'--log-level={log_level}'],
        on_exit=launch.actions.LogInfo(msg='Planner server exited')
    )
    
    # Controller server
    controller_server = Node(
        package='nav2_controller',
        executable='nav2_controller',
        name='controller_server',
        namespace=namespace,
        parameters=[
            {'use_sim_time': use_sim_time.lower() == 'true'},
            {'controller_frequency': 20.0},
            {'controller_plugins': ['FollowPath']},
            {'FollowPath.type': 'nav2_regulated_pure_pursuit_controller/RegulatedPurePursuitController'}
        ],
        arguments=[f'--log-level={log_level}'],
        on_exit=launch.actions.LogInfo(msg='Controller server exited')
    )
    
    nodes.extend([costmap_server, planner_server, controller_server])
    
    return nodes

def create_gui_nodes(context: LaunchContext) -> List:
    """Create GUI-related nodes"""
    namespace = LaunchConfiguration('namespace').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    log_level = LaunchConfiguration('log_level').perform(context)
    
    nodes = []
    
    # RViz2 visualization
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('my_robot_viz'),
        'rviz',
        'robot_navigation.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        namespace=namespace,
        arguments=['-d', rviz_config_file],
        parameters=[
            {'use_sim_time': use_sim_time.lower() == 'true'}
        ],
        arguments=[f'--log-level={log_level}'],
        on_exit=launch.actions.LogInfo(msg='RViz2 exited')
    )
    
    # Web interface node
    web_interface = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='web_interface',
        namespace=namespace,
        parameters=[
            {'use_sim_time': use_sim_time.lower() == 'true'},
            {'port': 9090}
        ],
        arguments=[f'--log-level={log_level}'],
        on_exit=launch.actions.LogInfo(msg='Web interface exited')
    )
    
    nodes.extend([rviz_node, web_interface])
    
    return nodes

def create_monitoring_nodes(context: LaunchContext) -> List:
    """Create monitoring and diagnostics"""
    namespace = LaunchConfiguration('namespace').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    
    nodes = []
    
    # Diagnostics aggregator
    diag_agg_config = PathJoinSubstitution([
        FindPackageShare('my_robot_diagnostics'),
        'config',
        'diagnostics_config.yaml'
    ])
    
    diag_aggregator = Node(
        package='diagnostic_aggregator',
        executable='aggregator_node',
        name='diagnostic_aggregator',
        namespace=namespace,
        parameters=[
            {'use_sim_time': use_sim_time.lower() == 'true'},
            {'analyzers': ['robot_systems']},
            {'path': diag_agg_config}
        ]
    )
    
    # Performance monitor
    perf_monitor = Node(
        package='my_robot_utils',
        executable='performance_monitor',
        name='performance_monitor',
        namespace=namespace,
        parameters=[
            {'use_sim_time': use_sim_time.lower() == 'true'}
        ]
    )
    
    nodes.extend([diag_aggregator, perf_monitor])
    
    return nodes

def create_error_handling_procedures(context: LaunchContext) -> List:
    """Create error handling and recovery procedures"""
    namespace = LaunchConfiguration('namespace').perform(context)
    
    # Event handlers for error recovery
    event_handlers = []
    
    # Example: Restart nodes on exit
    restart_on_exit = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=None,  # This would be attached to specific nodes
            on_exit=launch.actions.LogInfo(msg='Node exited, restarting...')
        )
    )
    
    # Example: Shutdown on critical error
    shutdown_on_critical = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=None,  # This would check for specific error conditions
            on_start=launch.actions.LogInfo(msg='Critical error detected, shutting down...')
        )
    )
    
    event_handlers.extend([restart_on_exit, shutdown_on_critical])
    
    return event_handlers

def create_logging_configuration(log_level: str) -> List:
    """Configure logging for all nodes"""
    # This would return actions to configure logging
    return []

# Launch file for parameter validation and management
def create_parameter_validation_launch():
    """Create launch file for parameter validation"""
    
    # Load parameter validation configuration
    validation_config_arg = DeclareLaunchArgument(
        'validation_config',
        default_value=[FindPackageShare('my_robot_params'), '/config/validation.yaml'],
        description='Parameter validation configuration file'
    )
    
    param_validator = Node(
        package='my_robot_params',
        executable='parameter_validator',
        name='parameter_validator',
        parameters=[
            {'config_file': LaunchConfiguration('validation_config')}
        ],
        arguments=['--validate-at-startup']
    )
    
    launch_description = LaunchDescription([
        validation_config_arg,
        param_validator
    ])
    
    return launch_description

# Conditional launch configuration
def create_conditional_launch():
    """Create launch configuration with complex conditions"""
    
    # Environment-based launch
    use_gpu_arg = DeclareLaunchArgument(
        'use_gpu',
        default_value=EnvironmentVariable(name='USE_GPU', default_value='false'),
        description='Use GPU acceleration'
    )
    
    # Machine-specific configuration
    machine_type_arg = DeclareLaunchArgument(
        'machine_type',
        default_value=EnvironmentVariable(name='MACHINE_TYPE', default_value='desktop'),
        choices=['desktop', 'jetson', 'raspberry', 'industrial'],
        description='Target machine type for hardware-specific optimizations'
    )
    
    def conditional_launch_setup(context: LaunchContext):
        """Setup launch based on conditions"""
        machine_type = LaunchConfiguration('machine_type').perform(context)
        use_gpu = LaunchConfiguration('use_gpu').perform(context)
        
        launch_actions = []
        
        # Add machine-specific optimizations
        if machine_type == 'jetson':
            # Add Jetson-specific nodes
            jetson_optimized_nodes = create_jetson_nodes(context)
            launch_actions.extend(jetson_optimized_nodes)
        
        elif machine_type == 'raspberry':
            # Add Raspberry Pi-specific nodes
            rpi_optimized_nodes = create_raspberry_nodes(context)
            launch_actions.extend(rpi_optimized_nodes)
        
        return launch_actions
    
    launch_description = LaunchDescription([
        use_gpu_arg,
        machine_type_arg,
        OpaqueFunction(function=conditional_launch_setup)
    ])
    
    return launch_description

def create_jetson_nodes(context: LaunchContext) -> List:
    """Create Jetson Nano/Xavier-specific nodes"""
    # Implementation would include hardware-optimized nodes
    return []

def create_raspberry_nodes(context: LaunchContext) -> List:
    """Create Raspberry Pi-specific nodes"""
    # Implementation would include lightweight nodes
    return []