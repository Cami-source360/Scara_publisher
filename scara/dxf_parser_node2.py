#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped, Pose
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
from nav_msgs.msg import Path

import ezdxf
import pathlib 
import math
import tf_transformations
import pandas as pd
import yaml 

from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32

class DXFParserNode(Node):
    def __init__(self):
        super().__init__('dxf_parser_node2')

        # Dictionary to map figure types to integers
        self.FIGURE_TYPE_MAP = {
            "LINE": 1,
            "LWPOLYLINE": 2,
            "POLYLINE": 3,
            "ARC": 4,
            "CIRCLE": 5,
            "SPLINE": 6
        }

        # Parameters
        self.declare_parameter('dxf_file', '/home/camila/ros2_ws_2502/src/scara/dxf_models/figura.dxf')
        dxf_path = self.get_parameter('dxf_file').get_parameter_value().string_value
        self.get_logger().info(f"Reading DXF file: {dxf_path}")
        
        # CSV output path
        self.CSV_PATH = '/home/camila/ros2_ws_2502/src/scara/csv_pointcloud/dxf_waypoints_v5.csv'
        
        # Publisher
        self.path_pub = self.create_publisher(Path, 'dxf_path', 10)
        self.pc_pub = self.create_publisher(PointCloud, 'dxf_pointcloud', 10)
        
        # Parse and publish waypoints
        self.waypoints = self.parse_dxf(dxf_path)
        self.export_to_csv()
        self.publish_waypoints()
        self.publish_pointcloud()



    def parse_dxf(self, path):
        """Parse DXF file and extract waypoints from geometric figures"""
        if not pathlib.Path(path).exists():
            self.get_logger().error(f"DXF file not found: {path}")
            return []

        try:
            dxf = ezdxf.readfile(path)
            ms = dxf.modelspace()
        except Exception as e:
            self.get_logger().error(f"Error reading DXF file: {e}")
            return []

        WP = []
        id = 0
        
        self.get_logger().info("Processing DXF geometric figures:")
        
        for e in ms:
            figure_type = e.dxftype()
            self.get_logger().info(f"- Processing: {figure_type} (ID: {id})")
            
            if figure_type == 'LINE':
                WP.append((e.dxf.start.x, e.dxf.start.y, 0.0, figure_type, id))
                WP.append((e.dxf.end.x, e.dxf.end.y, 0.0, figure_type, id))
                id += 1
                
            elif figure_type == 'LWPOLYLINE':
                for x, y, *_ in e.get_points():
                    WP.append((x, y, 0.0, figure_type, id))
                id += 1
                
            elif figure_type == 'POLYLINE':
                for p in e.points():
                    WP.append((p.x, p.y, 0.0, figure_type, id))
                id += 1
                
            elif figure_type == 'ARC':
                start_rad = math.radians(e.dxf.start_angle)
                end_rad = math.radians(e.dxf.end_angle)
                if end_rad < start_rad:
                    end_rad += 2 * math.pi
                
                for i in range(31):
                    angle = start_rad + i * (end_rad - start_rad) / 30
                    x = e.dxf.center.x + e.dxf.radius * math.cos(angle)
                    y = e.dxf.center.y + e.dxf.radius * math.sin(angle)
                    WP.append((x, y, 0.0, figure_type, id))
                id += 1
                
            elif figure_type == 'CIRCLE':
                for i in range(31):
                    angle = i * 2 * math.pi / 30
                    x = e.dxf.center.x + e.dxf.radius * math.cos(angle)
                    y = e.dxf.center.y + e.dxf.radius * math.sin(angle)
                    WP.append((x, y, 0.0, figure_type, id))
                id += 1
                
            elif figure_type == 'SPLINE':
                for vec in e.flattening(0.5):
                    WP.append((vec.x, vec.y, 0.0, figure_type, id))
                id += 1
                
            else:
                self.get_logger().warn(f"Unsupported figure type: {figure_type}")

        self.get_logger().info(f"Total waypoints extracted: {len(WP)}")
        return WP
    
    def _approximate_arc(self, cx, cy, radius, start_angle_deg, end_angle_deg, num_points=30):
        start_angle = math.radians(start_angle_deg)
        end_angle = math.radians(end_angle_deg)
        if end_angle < start_angle:
            end_angle += 2 * math.pi

        arc_points = []
        for i in range(num_points + 1):
            angle = start_angle + i * (end_angle - start_angle) / num_points
            x = cx + radius * math.cos(angle)
            y = cy + radius * math.sin(angle)
            arc_points.append(Point(x=x, y=y, z=0.0))
        return arc_points

    def publish_waypoints(self):
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for i, wp in enumerate(self.waypoints):
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = wp[0] / 100.0
            pose.pose.position.y = wp[1] / 100.0
            pose.pose.position.z = wp[2] / 100.0

            # Guardamos figure_id y type en orientation
            figure_type = wp[3]
            figure_id = wp[4]

            pose.pose.orientation.x = float(figure_id)  
            pose.pose.orientation.y = float(self.FIGURE_TYPE_MAP.get(figure_type, 0))  
            pose.pose.orientation.z = 0.0  
            pose.pose.orientation.w = 1.0  

            path_msg.poses.append(pose)

        self.get_logger().info(f"Publishing path with {len(path_msg.poses)} poses.")
        self.path_pub.publish(path_msg)

    def publish_pointcloud(self):
        pc = PointCloud()
        pc.header.frame_id = "map"
        pc.header.stamp = self.get_clock().now().to_msg()

        for wp in self.waypoints:
            pc.points.append(Point32(x=wp[0]/10.0, y=wp[1]/10.0, z=wp[2]/10.0))
            
        self.pc_pub.publish(pc)

    def export_to_csv(self, filename=None):
        try:
            if filename is None:
                filename = self.CSV_PATH
            
            data = {'x': [], 'y': [], 'z': [], 'figure_type': [], 'figure_id': []}
            
            for point in self.waypoints:
                data['x'].append(point[0])
                data['y'].append(point[1])
                data['z'].append(point[2])
                data['figure_type'].append(point[3])
                data['figure_id'].append(point[4])
            
            df = pd.DataFrame(data)
            df.to_csv(filename, index=False)
            self.get_logger().info(f"Waypoints exported to CSV at: {filename}")
        except Exception as e:
            self.get_logger().error(f"Failed to write CSV: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = DXFParserNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()