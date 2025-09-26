#!/usr/bin/env python3

"""
SCARA Controller Node

This node subscribes to joint commands and publishes control signals for a SCARA robot.
It demonstrates basic ROS2 subscriber/publisher functionality for robot control.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist, Point
import math


class ScaraControllerNode(Node):
    """A ROS2 node that controls SCARA robot based on received commands."""
    
    def __init__(self):
        super().__init__('scara_controller_node')
        
        # Declare parameters
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('max_joint_velocity', 1.0)
        self.declare_parameter('position_tolerance', 0.01)
        
        # Get parameters
        self.control_rate = self.get_parameter('control_rate').get_parameter_value().double_value
        self.max_joint_velocity = self.get_parameter('max_joint_velocity').get_parameter_value().double_value
        self.position_tolerance = self.get_parameter('position_tolerance').get_parameter_value().double_value
        
        # Subscribers
        self.joint_command_subscriber = self.create_subscription(
            Float64MultiArray,
            'joint_commands',
            self.joint_command_callback,
            10
        )
        
        self.cartesian_command_subscriber = self.create_subscription(
            Point,
            'cartesian_commands',
            self.cartesian_command_callback,
            10
        )
        
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )
        
        # Publishers
        self.motor_command_publisher = self.create_publisher(
            Float64MultiArray,
            'motor_commands',
            10
        )
        
        self.control_status_publisher = self.create_publisher(
            JointState,
            'control_status',
            10
        )
        
        # Control timer
        timer_period = 1.0 / self.control_rate
        self.control_timer = self.create_timer(timer_period, self.control_loop)
        
        # Internal state
        self.target_joint_positions = [0.0, 0.0, 0.0, 0.0]
        self.current_joint_positions = [0.0, 0.0, 0.0, 0.0]
        self.joint_velocities = [0.0, 0.0, 0.0, 0.0]
        self.control_mode = 'position'  # 'position' or 'velocity'
        
        self.get_logger().info(f'SCARA Controller Node started with rate: {self.control_rate} Hz')
    
    def joint_command_callback(self, msg):
        """Callback for joint position commands."""
        if len(msg.data) >= 4:
            self.target_joint_positions = list(msg.data[:4])
            self.control_mode = 'position'
            self.get_logger().debug(f'Received joint commands: {self.target_joint_positions}')
    
    def cartesian_command_callback(self, msg):
        """Callback for cartesian position commands."""
        # Convert cartesian coordinates to joint angles using inverse kinematics
        target_joints = self.inverse_kinematics(msg.x, msg.y, msg.z)
        if target_joints:
            self.target_joint_positions = target_joints
            self.control_mode = 'position'
            self.get_logger().info(f'Cartesian command -> Joint targets: {target_joints}')
    
    def joint_state_callback(self, msg):
        """Callback for current joint states."""
        if len(msg.position) >= 4:
            self.current_joint_positions = list(msg.position[:4])
        if len(msg.velocity) >= 4:
            self.joint_velocities = list(msg.velocity[:4])
    
    def inverse_kinematics(self, x, y, z):
        """
        Simple inverse kinematics for SCARA robot.
        Returns joint angles [q1, q2, q3, q4] or None if unreachable.
        """
        # Link lengths
        L1 = 0.3
        L2 = 0.2
        
        # Check if point is reachable
        r = math.sqrt(x*x + y*y)
        if r > (L1 + L2) or r < abs(L1 - L2):
            self.get_logger().warn(f'Point ({x:.3f}, {y:.3f}, {z:.3f}) is unreachable')
            return None
        
        # Calculate joint angles
        try:
            # Joint 2 (elbow)
            cos_q2 = (r*r - L1*L1 - L2*L2) / (2 * L1 * L2)
            cos_q2 = max(-1.0, min(1.0, cos_q2))  # Clamp to valid range
            q2 = math.acos(cos_q2)
            
            # Joint 1 (shoulder)
            alpha = math.atan2(y, x)
            beta = math.atan2(L2 * math.sin(q2), L1 + L2 * math.cos(q2))
            q1 = alpha - beta
            
            # Joint 3 (vertical - prismatic)
            q3 = z
            
            # Joint 4 (wrist rotation) - keep current value
            q4 = self.current_joint_positions[3]
            
            return [q1, q2, q3, q4]
            
        except (ValueError, ZeroDivisionError) as e:
            self.get_logger().error(f'Inverse kinematics calculation failed: {e}')
            return None
    
    def control_loop(self):
        """Main control loop."""
        if self.control_mode == 'position':
            self.position_control()
        
        # Publish control status
        self.publish_control_status()
    
    def position_control(self):
        """Simple position control with velocity limiting."""
        motor_commands = Float64MultiArray()
        motor_commands.data = []
        
        for i in range(4):
            # Calculate position error
            error = self.target_joint_positions[i] - self.current_joint_positions[i]
            
            # Simple proportional control
            kp = 2.0  # Proportional gain
            desired_velocity = kp * error
            
            # Limit velocity
            if abs(desired_velocity) > self.max_joint_velocity:
                desired_velocity = math.copysign(self.max_joint_velocity, desired_velocity)
            
            motor_commands.data.append(desired_velocity)
        
        self.motor_command_publisher.publish(motor_commands)
    
    def publish_control_status(self):
        """Publish current control status."""
        status = JointState()
        status.header.stamp = self.get_clock().now().to_msg()
        status.header.frame_id = 'control_status'
        
        status.name = ['joint1', 'joint2', 'joint3', 'joint4']
        status.position = self.current_joint_positions
        status.velocity = self.joint_velocities
        
        # Calculate position errors
        errors = [
            self.target_joint_positions[i] - self.current_joint_positions[i]
            for i in range(4)
        ]
        status.effort = errors  # Use effort field for errors
        
        self.control_status_publisher.publish(status)
    
    def is_at_target(self):
        """Check if robot is at target position within tolerance."""
        for i in range(4):
            error = abs(self.target_joint_positions[i] - self.current_joint_positions[i])
            if error > self.position_tolerance:
                return False
        return True


def main(args=None):
    """Main function to initialize and run the SCARA controller node."""
    rclpy.init(args=args)
    
    scara_controller = ScaraControllerNode()
    
    try:
        rclpy.spin(scara_controller)
    except KeyboardInterrupt:
        scara_controller.get_logger().info('SCARA Controller Node stopped by user')
    finally:
        scara_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()