import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile
import math
import numpy as np
from icecream import ic
from sensor_msgs.msg import JointState # referencia


class ScaraInverseKinematics(Node):
    def __init__(self):
        super().__init__('scara_inverse_kinematics')
        
        # ---- Parameters (declare + get) ----
        self.declare_parameter('publish_rate', 50.0)      # Hz
        self.declare_parameter('r1', 15.5)                 # Length of first arm (cm)
        self.declare_parameter('r2', 13.0)                 # Length of second arm (cm)

        self.rate_hz = float(self.get_parameter('publish_rate').value)
        self.l1 = float(self.get_parameter('r1').value)
        self.l2 = float(self.get_parameter('r2').value)
        
        # ---- State ----
        # Desired position (initially in mm, converted to cm for calculations)
        self.px = 0.0  # cm
        self.py = 0.0  # cm
        self.pz = 0.0  # mm (no conversion needed for z)
        
        # Output joint configuration
        self.th1 = 0.0  # rad
        self.th2 = 0.0  # rad
        self.flag = False 

        # ---- Pub/Sub ----
        qos = QoSProfile(depth=10)
        self.create_subscription(Twist, 'twist_mux_out', self._desired_pos_callback, qos)
        
        
        self.publisher_inv_kin = self.create_publisher(Twist, 'inv_kin', qos)
        
        
        # ---- Timer ----
        self.last_time = None
        self.dt = 1.0/self.rate_hz
        self.timer = self.create_timer(self.dt, self.publish_inverse_kinematics)
        
        self.get_logger().info(f'SCARA Inverse Kinematics node started')
        self.get_logger().info(f'Robot parameters: r1={self.l1}cm, r2={self.l2}cm')

    def _2r_inverse_kinematics(self):
        """
        Inverse kinematics for a 2R robot manipulator
        px, py are the desired end-effector positions (in cm)
        th1, th2 are the resulting joint variables (angles in rad)
        """
        ic(self.px, self.py)
        ic(self.l1, self.l2)
    
        r = np.sqrt(np.power(self.px,2) + np.power(self.py,2))
        d = (np.power(r,2) - np.power(self.l1,2) - np.power(self.l2,2))/(2*self.l1*self.l2)
        
        theta_2 = -np.arctan2(np.sqrt(1 - np.power(d,2)),d)
        theta_1 = np.arctan2(self.py,self.px) - np.arctan2(np.sin(theta_2)*self.l2,(self.l1 + np.cos(theta_2)*self.l2))


        # now = self.get_clock().now()
        # js = JointState()
        # js.header.stamp = now.to_msg()
        # js.name = ["brazo1_joint", "brazo2_joint", "prismatic_joint"]
        # js.position = [theta_1, theta_2, 0.]
        # # js.velocity = [w_l, w_r]
        # self.pub_js.publish(js)

        self.th1 = theta_1
        self.th2 = theta_2
        
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

    # def _desired_pos_callback_stm(self, msg: PoseStamped):
    #         """
    #         Callback for desired position topic
    #         Converts input from mm to cm for x,y and keeps z in mm
    #         """
    #         # Convert from mm to cm for x and y coordinates
    #         self.px = msg.pose.position.x * 100.0  # m to cm
    #         self.py = msg.pose.position.y * 100.0  # m to cm
    #         self.pz = msg.pose.position.z * 100.0         # keep z in mm
            
    #         self.get_logger().debug(f'Received desired position: x={self.px:.2f}cm, y={self.py:.2f}cm, z={self.pz:.2f}mm')
            
    #         if not self.flag:
    #             self.flag = True


    def publish_inverse_kinematics(self):
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
    node = ScaraInverseKinematics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()