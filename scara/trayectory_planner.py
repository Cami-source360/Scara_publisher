import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile
import math
import numpy as np
import pandas as pd
from icecream import ic
from sensor_msgs.msg import JointState # referencia


class TrayectorPlannerNode(Node):
    def __init__(self):
        super().__init__('scara_inverse_kinematics')
        
        # ---- Parameters (declare + get) ----
        self.declare_parameter('publish_rate', 50.0)      # Hz
                      # Length of second arm (cm)

        self.rate_hz = float(self.get_parameter('publish_rate').value)

        self.csv_waypoints = pd.read_csv('/home/camila/ros2_ws_2502/src/scara/csv_pointcloud/dxf_waypoints_v5.csv')
        # ---- Pub/Sub ----
        qos = QoSProfile(depth=10)
       
        self.publisher_trayectories = self.create_publisher(Twist, 'trayectories', qos)
        
        
        # ---- Timer ----
        self.last_time = None
        self.dt = 1.0/self.rate_hz
        self.timer = self.create_timer(self.dt, self.publisher_trayectories)
        
        self.get_logger().info(f'Trayectory Planner node started')
        self.get_logger().info(f'Robot parameters: r1={self.l1}cm, r2={self.l2}cm')

    def coeficients_spline(self):
        "calcula los coeficientes del spline quintico"
        #en tiempo 0
         



    def trayectory_planner(self):
        "lee los datos del archivo dxf_waypoints.csv y les aplica un spline cubico, se guardan estas variables en "
        "un "
        for index, row in self.csv_waypoints.iterrows():
            self.px = row['x']*100
            self.py = row['y']*100
            self.pz = row['z']*100
            
            self.get_logger().info(f'leyendo datos del csv: x={self.px:.2f}cm, y={self.py:.2f}cm, z={self.pz:.2f}mm')

            # Aplicar spline cúbico aquí y guardar resultados en variables
        #coeficients:
        
           
          
        
    def _desired_pos_callback(self, msg: Twist):
        """
        Callback for desired position topic
        Converts input from mm to cm for x,y and keeps z in mm
        """
        # Convert from mm to cm for x and y coordinates
        self.px = msg.linear.x * 100.0  # m to cm
        self.py = msg.linear.y * 100.0  # m to cm
        self.pz = msg.linear.z * 100.0        # keep z in cm
        
        self.get_logger().debug(f'Received desired position: x={self.px:.2f}cm, y={self.py:.2f}cm, z={self.pz:.2f}mm')
        
        if not self.flag:
            self.flag = True




    def publisher_trayectories(self):
        """
        Timer callback to compute and publish inverse kinematics
        """
        now = self.get_clock().now()
        if self.last_time is None:
            self.last_time = now
            return

        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        if dt <= 0.0:
            return

        if self.flag:
            # Calculate inverse kinematics
            self._2r_inverse_kinematics()
            
            # Create and publish Twist message
            tw = Twist()
            tw.linear.x = (self.th1)  # Convert to degrees
            tw.linear.y = (self.th2)  # Convert to degrees  
            tw.linear.z = self.pz                 # Keep z in cm
            
            self.publisher_inv_kin.publish(tw)
            
            self.get_logger().debug(f'Published joint angles: th1={math.degrees(self.th1):.2f}°, th2={math.degrees(self.th2):.2f}°, z={self.pz:.2f}mm')


def main(args=None):
    rclpy.init(args=args)
    node = TrayectorPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()