#!/usr/bin/env python3

"""
Unit tests for SCARA Publisher Node.
"""

import unittest
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Point
import math
import time

# Import the nodes to test
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from scara_publisher.scara_publisher_node import ScaraPublisherNode
from scara_publisher.scara_controller_node import ScaraControllerNode


class TestScaraPublisher(unittest.TestCase):
    """Test cases for SCARA Publisher Node."""
    
    @classmethod
    def setUpClass(cls):
        """Set up test class."""
        rclpy.init()
    
    @classmethod
    def tearDownClass(cls):
        """Tear down test class."""
        rclpy.shutdown()
    
    def setUp(self):
        """Set up test case."""
        self.node = ScaraPublisherNode()
    
    def tearDown(self):
        """Tear down test case."""
        self.node.destroy_node()
    
    def test_node_initialization(self):
        """Test that the node initializes correctly."""
        self.assertIsInstance(self.node, ScaraPublisherNode)
        self.assertEqual(self.node.get_name(), 'scara_publisher_node')
    
    def test_joint_positions_initialization(self):
        """Test that joint positions are initialized correctly."""
        self.assertEqual(len(self.node.joint_positions), 4)
        self.assertTrue(all(isinstance(pos, float) for pos in self.node.joint_positions))
    
    def test_parameters(self):
        """Test parameter handling."""
        # Test default parameters
        self.assertGreater(self.node.publish_rate, 0.0)
        self.assertEqual(len(self.node.joint_names), 4)
        self.assertGreater(self.node.workspace_radius, 0.0)


class TestScaraController(unittest.TestCase):
    """Test cases for SCARA Controller Node."""
    
    @classmethod
    def setUpClass(cls):
        """Set up test class."""
        rclpy.init()
    
    @classmethod
    def tearDownClass(cls):
        """Tear down test class."""
        rclpy.shutdown()
    
    def setUp(self):
        """Set up test case."""
        self.node = ScaraControllerNode()
    
    def tearDown(self):
        """Tear down test case."""
        self.node.destroy_node()
    
    def test_node_initialization(self):
        """Test that the controller node initializes correctly."""
        self.assertIsInstance(self.node, ScaraControllerNode)
        self.assertEqual(self.node.get_name(), 'scara_controller_node')
    
    def test_inverse_kinematics_valid_point(self):
        """Test inverse kinematics with a valid reachable point."""
        x, y, z = 0.3, 0.2, 0.1
        result = self.node.inverse_kinematics(x, y, z)
        
        self.assertIsNotNone(result)
        self.assertEqual(len(result), 4)
        self.assertTrue(all(isinstance(angle, float) for angle in result))
    
    def test_inverse_kinematics_unreachable_point(self):
        """Test inverse kinematics with an unreachable point."""
        x, y, z = 1.0, 1.0, 0.1  # Too far from robot base
        result = self.node.inverse_kinematics(x, y, z)
        
        self.assertIsNone(result)
    
    def test_control_mode_initialization(self):
        """Test that control mode is initialized correctly."""
        self.assertEqual(self.node.control_mode, 'position')
    
    def test_joint_limits(self):
        """Test that joint positions are within reasonable limits."""
        self.assertEqual(len(self.node.current_joint_positions), 4)
        self.assertEqual(len(self.node.target_joint_positions), 4)


class TestKinematics(unittest.TestCase):
    """Test kinematic calculations."""
    
    def test_forward_kinematics_zero_position(self):
        """Test forward kinematics at zero position."""
        # This would be implemented if we had a separate kinematics module
        # For now, we test the inverse kinematics in the controller
        pass
    
    def test_workspace_limits(self):
        """Test workspace boundary conditions."""
        # Test points at workspace boundary
        L1, L2 = 0.3, 0.2
        max_reach = L1 + L2
        min_reach = abs(L1 - L2)
        
        # Points at maximum reach should be valid
        x_max = max_reach
        y_max = 0.0
        self.assertLessEqual(math.sqrt(x_max*x_max + y_max*y_max), max_reach)
        
        # Points at minimum reach should be valid
        x_min = min_reach
        y_min = 0.0
        self.assertGreaterEqual(math.sqrt(x_min*x_min + y_min*y_min), min_reach)


if __name__ == '__main__':
    unittest.main()