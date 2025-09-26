#!/usr/bin/env python3

"""
SCARA Publisher Node

This node publishes joint states and commands for a SCARA (Selective Compliance Assembly Robot Arm) robot.
It demonstrates basic ROS2 publisher functionality that can be used as a foundation for SCARA robot control.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Header, Float64MultiArray
import math
import time


class ScaraPublisherNode(Node):
    """A ROS2 node that publishes SCARA robot joint states and commands."""
    
    def __init__(self):
        super().__init__('scara_publisher_node')
        
        # Declare parameters
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('joint_names', ['joint1', 'joint2', 'joint3', 'joint4'])
        self.declare_parameter('workspace_radius', 0.5)
        
        # Get parameters
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.joint_names = self.get_parameter('joint_names').get_parameter_value().string_array_value
        self.workspace_radius = self.get_parameter('workspace_radius').get_parameter_value().double_value
        
        # Publishers
        self.joint_state_publisher = self.create_publisher(
            JointState,
            'joint_states',
            10
        )
        
        self.joint_command_publisher = self.create_publisher(
            Float64MultiArray,
            'joint_commands',
            10
        )
        
        self.end_effector_publisher = self.create_publisher(
            Point,
            'end_effector_position',
            10
        )
        
        # Timer for publishing
        timer_period = 1.0 / self.publish_rate  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Internal state
        self.joint_positions = [0.0, 0.0, 0.0, 0.0]  # 4 joints for SCARA
        self.time_counter = 0.0
        
        self.get_logger().info(f'SCARA Publisher Node started with rate: {self.publish_rate} Hz')
        self.get_logger().info(f'Joint names: {self.joint_names}')
    
    def timer_callback(self):
        """Timer callback to publish joint states and commands."""
        # Generate sinusoidal joint movements for demonstration
        self.time_counter += 1.0 / self.publish_rate
        
        # Base rotation (joint 1)
        self.joint_positions[0] = 0.5 * math.sin(0.5 * self.time_counter)
        
        # Shoulder joint (joint 2)
        self.joint_positions[1] = 0.3 * math.cos(0.3 * self.time_counter)
        
        # Elbow joint (joint 3)
        self.joint_positions[2] = 0.2 * math.sin(0.7 * self.time_counter)
        
        # Wrist rotation (joint 4)
        self.joint_positions[3] = 0.8 * math.cos(0.4 * self.time_counter)
        
        # Publish joint states
        self.publish_joint_states()
        
        # Publish joint commands
        self.publish_joint_commands()
        
        # Calculate and publish end effector position
        self.publish_end_effector_position()
    
    def publish_joint_states(self):
        """Publish current joint states."""
        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.header.frame_id = 'base_link'
        
        joint_state.name = self.joint_names
        joint_state.position = self.joint_positions
        joint_state.velocity = [0.0] * len(self.joint_names)
        joint_state.effort = [0.0] * len(self.joint_names)
        
        self.joint_state_publisher.publish(joint_state)
    
    def publish_joint_commands(self):
        """Publish joint commands for the robot."""
        joint_commands = Float64MultiArray()
        joint_commands.data = self.joint_positions
        
        self.joint_command_publisher.publish(joint_commands)
    
    def publish_end_effector_position(self):
        """Calculate and publish end effector position based on forward kinematics."""
        # Simplified SCARA forward kinematics
        # Assuming link lengths: L1=0.3m, L2=0.2m
        L1 = 0.3
        L2 = 0.2
        
        q1 = self.joint_positions[0]  # Base rotation
        q2 = self.joint_positions[1]  # Shoulder
        q3 = self.joint_positions[2]  # Elbow
        
        # Forward kinematics calculation
        x = L1 * math.cos(q1) + L2 * math.cos(q1 + q2)
        y = L1 * math.sin(q1) + L2 * math.sin(q1 + q2)
        z = q3  # Prismatic joint for vertical movement
        
        end_effector_pos = Point()
        end_effector_pos.x = x
        end_effector_pos.y = y
        end_effector_pos.z = z
        
        self.end_effector_publisher.publish(end_effector_pos)


def main(args=None):
    """Main function to initialize and run the SCARA publisher node."""
    rclpy.init(args=args)
    
    scara_publisher = ScaraPublisherNode()
    
    try:
        rclpy.spin(scara_publisher)
    except KeyboardInterrupt:
        scara_publisher.get_logger().info('SCARA Publisher Node stopped by user')
    finally:
        scara_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()