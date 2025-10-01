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
import csv 
import yaml  # Add this import at the top
import pandas as pd

from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32

class DXFParserNode(Node):
    def __init__(self):
        super().__init__('dxf_parser_node')

        # Parameters
        self.declare_parameter('dxf_file', '/home/camila/ros2_ws_2502/src/scara/dxf_models/trayectroria.dxf')
        dxf_path = self.get_parameter('dxf_file').get_parameter_value().string_value
        self.get_logger().info(f"Reading DXF file: {dxf_path}")
        self.CSV_PATH = '/home/camila/ros2_ws_2502/src/scara/csv_pointcloud/dxf_waypoints_v5.csv'
          # Publisher
        self.path_pub = self.create_publisher(Path, 'dxf_path', 10)
        self.pc_pub = self.create_publisher(PointCloud, 'dxf_pointcloud', 10)
        

        # Parse and publish waypoints
        self.waypoints = self.parse_dxf(dxf_path)
        self.export_to_csv()  # <--- Export to CSV
        self.publish_waypoints()
        self.publish_pointcloud()

    def parse_dxf(self, path):
        if not pathlib.Path(path).exists():
            self.get_logger().error(f"File does not exist: {path}")
            return []

        doc = ezdxf.readfile(path)
        msp = doc.modelspace()

        waypoints = []

        for e in msp:
            if e.dxftype() == 'LINE':
                waypoints.extend([
                    Point(x=e.dxf.start.x, y=e.dxf.start.y, z=0.0),
                    Point(x=e.dxf.end.x, y=e.dxf.end.y, z=0.0)
                ])
                self.get_logger().info(f"Publishing LINE with {len(waypoints)} poses.")

            elif e.dxftype() == 'LWPOLYLINE':
                for x, y, *_ in e.get_points():
                    waypoints.append(Point(x=x, y=y, z=0.0))
                    self.get_logger().info(f"Publishing LWPOLYLINE with {len(waypoints)} poses.")

            elif e.dxftype() == 'ARC':
                waypoints.extend(self._approximate_arc(
                    e.dxf.center.x,
                    e.dxf.center.y,
                    e.dxf.radius,
                    e.dxf.start_angle,
                    e.dxf.end_angle
                ))
                self.get_logger().info(f"Publishing ARC with {len(waypoints)} poses.")
                
            elif e.dxftype() == 'CIRCLE':
                waypoints.extend(self._approximate_arc(
                    e.dxf.center.x,
                    e.dxf.center.y,
                    e.dxf.radius,
                    0.0,
                    360.0
                ))
                self.get_logger().info(f"Publishing CIRCLE with {len(waypoints)} poses.")

            elif e.dxftype() == 'SPLINE':
                for vec in e.flattening(0.5):  # 0.5 is the tolerance in drawing units
                    waypoints.append(Point(x=vec.x, y=vec.y, z=0.0))

        self.get_logger().info(f"Extracted {len(waypoints)} waypoints from DXF.")
        return waypoints

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
        path_msg.header.frame_id = "map"  # Change if needed
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for i, wp in enumerate(self.waypoints):
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position = wp
            pose.pose.orientation.w = 1.0  # Identity quaternion
            path_msg.poses.append(pose)

        self.get_logger().info(f"Publishing path with {len(path_msg.poses)} poses.")
        self.path_pub.publish(path_msg)


    def publish_pointcloud(self):
        pc = PointCloud()
        pc.header.frame_id = "map"
        pc.header.stamp = self.get_clock().now().to_msg()

        for wp in self.waypoints:
            pc.points.append(Point32(x=wp.x/10., y=wp.y/10., z=wp.z/10.))  # Convert mm to meters
            

        self.pc_pub.publish(pc)

    def export_to_csv(self, filename='dxf_waypoints_v5.csv'):
        try:
            with open(filename, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(['x', 'y', 'z'])  # Header
                for point in self.waypoints:
                    writer.writerow([point.x, point.y, point.z])
            self.get_logger().info(f"Waypoints exported to CSV at: {filename}")
        except Exception as e:
            self.get_logger().error(f"Failed to write CSV: {e}")

    # def export_to_csv(self, filename=None):
    #     try:
    #         if filename is None:
    #             filename = self.CSV_PATH
    #         time = 0.0
    #         data = {'x': [], 'y': [], 'z': [], 'time': []}
    #         for point in self.waypoints:
    #             data['x'].append(point.x)
    #             data['y'].append(point.y)
    #             data['z'].append(point.z)
    #             data['time'].append(time)
    #             time += 0.1  # Increment time for each waypoint
    #         df = pd.DataFrame(data)
    #         df.to_csv(filename, index=False)
    #         self.get_logger().info(f"Waypoints exported to CSV at: {filename}")
    #     except Exception as e:
    #         self.get_logger().error(f"Failed to write CSV: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = DXFParserNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
