import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile
import math
import numpy as np
from icecream import ic
from sensor_msgs.msg import JointState # referencia


class TranslateNode(Node):
    def __init__(self):
        super().__init__('translate_node')
        
        # ---- Parameters (declare + get) ----
        self.declare_parameter('publish_rate', 50.0)      # Hz
        self.rate_hz = float(self.get_parameter('publish_rate').value)
        # ---- State ----
        # Desired position (initially in mm, converted to cm for calculations)
        self.px = 0.0  # cm
        self.py = 0.0  # cm
        self.pz = 0.0  # mm (no conversion needed for z)
        

        # ---- Pub/Sub ----
        qos = QoSProfile(depth=10)
        
        self.create_subscription(PoseStamped, 'goal_pose', self._desired_pos_callback_stm, qos)
        
        self.publisher_type_twist = self.create_publisher(Twist, 'goal_pose_twist', qos)
    
        
        # ---- Timer ----
        self.last_time = None
        self.dt = 1.0/self.rate_hz
        self.timer = self.create_timer(self.dt, self.translate_func)
        
        self.get_logger().info(f'SCARA Inverse Kinematics node started')
        self.get_logger().info(f'Robot parameters: r1={self.l1}cm, r2={self.l2}cm')

   


    def _desired_pos_callback_stm(self, msg: PoseStamped):
            """
            Callback for desired position topic
            Converts input from mm to cm for x,y and keeps z in mm
            """
            # Convert from mm to cm for x and y coordinates
            self.px = msg.pose.position.x * 100.0  # m to cm
            self.py = msg.pose.position.y * 100.0  # m to cm
            self.pz = msg.pose.position.z * 100.0         # keep z in mm
            
            self.get_logger().debug(f'Received desired position: x={self.px:.2f}cm, y={self.py:.2f}cm, z={self.pz:.2f}mm')
            
            if not self.flag:
                self.flag = True

            
    def translate_func(self):
        """
        Timer callback to compute and publish the translated Twist message
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

            # Create and publish Twist message
            tw = Twist()
            tw.linear.x = self.px # Convert to degrees
            tw.linear.y = self.py  # Convert to degrees  
            tw.linear.z = self.pz                 # Keep z in mm
            
            self.publisher_type_twist.publish(tw)
            
            

def main(args=None):
    rclpy.init(args=args)
    node = TranslateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()