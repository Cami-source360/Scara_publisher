#!/usr/bin/env python3

"""
Basic usage example for SCARA Publisher package.

This script demonstrates how to interact with the SCARA robot nodes
programmatically without ROS2 if needed for testing or simulation.
"""

import math
import time
import sys
import os

# Add the package to the path for standalone testing
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))


def demonstrate_kinematics():
    """Demonstrate kinematic calculations."""
    print("=== SCARA Robot Kinematics Demo ===\n")
    
    # Robot parameters
    L1 = 0.3  # Link 1 length (meters)
    L2 = 0.2  # Link 2 length (meters)
    
    print(f"Robot Parameters:")
    print(f"  Link 1 length: {L1}m")
    print(f"  Link 2 length: {L2}m")
    print(f"  Maximum reach: {L1 + L2}m")
    print(f"  Minimum reach: {abs(L1 - L2)}m\n")
    
    # Forward kinematics example
    print("Forward Kinematics Examples:")
    joint_configurations = [
        [0.0, 0.0, 0.0, 0.0],      # Home position
        [0.5, 0.3, 0.1, 0.8],     # Sample position 1
        [1.0, -0.5, -0.1, -0.5],  # Sample position 2
    ]
    
    for i, joints in enumerate(joint_configurations):
        q1, q2, q3, q4 = joints
        
        # Calculate end effector position
        x = L1 * math.cos(q1) + L2 * math.cos(q1 + q2)
        y = L1 * math.sin(q1) + L2 * math.sin(q1 + q2)
        z = q3  # Prismatic joint
        
        print(f"  Config {i+1}: Joints={joints}")
        print(f"             Position=({x:.3f}, {y:.3f}, {z:.3f})")
        print(f"             Reach={math.sqrt(x*x + y*y):.3f}m\n")
    
    # Inverse kinematics example
    print("Inverse Kinematics Examples:")
    target_positions = [
        (0.4, 0.2, 0.1),    # Reachable point
        (0.3, 0.0, 0.05),   # Straight ahead
        (0.6, 0.0, 0.0),    # Edge of workspace (might be unreachable)
    ]
    
    for i, (x, y, z) in enumerate(target_positions):
        r = math.sqrt(x*x + y*y)
        
        if r > (L1 + L2) or r < abs(L1 - L2):
            print(f"  Target {i+1}: Position=({x}, {y}, {z}) - UNREACHABLE")
            continue
        
        # Calculate joint angles
        try:
            cos_q2 = (r*r - L1*L1 - L2*L2) / (2 * L1 * L2)
            cos_q2 = max(-1.0, min(1.0, cos_q2))
            q2 = math.acos(cos_q2)
            
            alpha = math.atan2(y, x)
            beta = math.atan2(L2 * math.sin(q2), L1 + L2 * math.cos(q2))
            q1 = alpha - beta
            
            q3 = z  # Prismatic joint
            q4 = 0.0  # Wrist rotation (arbitrary)
            
            joints = [q1, q2, q3, q4]
            
            print(f"  Target {i+1}: Position=({x}, {y}, {z})")
            print(f"              Joints=[{q1:.3f}, {q2:.3f}, {q3:.3f}, {q4:.3f}]")
            
            # Verify with forward kinematics
            x_check = L1 * math.cos(q1) + L2 * math.cos(q1 + q2)
            y_check = L1 * math.sin(q1) + L2 * math.sin(q1 + q2)
            z_check = q3
            
            error = math.sqrt((x-x_check)**2 + (y-y_check)**2 + (z-z_check)**2)
            print(f"              Verification error: {error:.6f}m\n")
            
        except (ValueError, ZeroDivisionError) as e:
            print(f"  Target {i+1}: Position=({x}, {y}, {z}) - CALCULATION ERROR: {e}\n")


def demonstrate_trajectory():
    """Demonstrate trajectory generation."""
    print("=== Trajectory Generation Demo ===\n")
    
    # Generate a circular trajectory
    radius = 0.3
    center_x = 0.3
    center_y = 0.0
    height = 0.1
    
    print(f"Circular trajectory:")
    print(f"  Center: ({center_x}, {center_y})")
    print(f"  Radius: {radius}m")
    print(f"  Height: {height}m\n")
    
    print("Time(s)  X(m)    Y(m)    Z(m)    Angle(rad)")
    print("-" * 45)
    
    duration = 10.0  # seconds
    steps = 20
    
    for i in range(steps + 1):
        t = i * duration / steps
        angle = 2 * math.pi * t / duration
        
        x = center_x + radius * math.cos(angle)
        y = center_y + radius * math.sin(angle)
        z = height + 0.05 * math.sin(4 * angle)  # Small vertical oscillation
        
        print(f"{t:6.1f}   {x:6.3f}  {y:6.3f}  {z:6.3f}   {angle:8.3f}")


def demonstrate_workspace():
    """Demonstrate workspace analysis."""
    print("\n=== Workspace Analysis ===\n")
    
    L1, L2 = 0.3, 0.2
    
    # Calculate workspace boundaries
    max_reach = L1 + L2
    min_reach = abs(L1 - L2)
    
    print(f"Workspace Analysis:")
    print(f"  Maximum reach: {max_reach}m")
    print(f"  Minimum reach: {min_reach}m")
    print(f"  Workspace area: {math.pi * (max_reach**2 - min_reach**2):.3f} m²")
    print()
    
    # Test various points
    test_points = [
        (0.0, 0.0),     # Origin
        (0.1, 0.0),     # Inside minimum reach
        (0.15, 0.0),    # At minimum reach
        (0.3, 0.0),     # Mid workspace
        (0.5, 0.0),     # At maximum reach
        (0.6, 0.0),     # Outside workspace
        (0.3, 0.3),     # Diagonal point
        (0.4, 0.4),     # Corner test
    ]
    
    print("Point Reachability Analysis:")
    print("X(m)   Y(m)   Distance(m)  Reachable?")
    print("-" * 38)
    
    for x, y in test_points:
        distance = math.sqrt(x*x + y*y)
        reachable = min_reach <= distance <= max_reach
        status = "YES" if reachable else "NO"
        print(f"{x:5.1f}  {y:5.1f}  {distance:10.3f}  {status:>9}")


def main():
    """Main demonstration function."""
    print("SCARA Publisher Package - Usage Examples")
    print("=" * 50)
    print()
    
    try:
        demonstrate_kinematics()
        demonstrate_trajectory()
        demonstrate_workspace()
        
        print("\n" + "=" * 50)
        print("Demo completed successfully!")
        print("To run the actual ROS2 nodes, use:")
        print("  ros2 launch scara_publisher scara_publisher_launch.py")
        print()
        
    except Exception as e:
        print(f"Error during demonstration: {e}")
        return 1
    
    return 0


if __name__ == '__main__':
    exit(main())