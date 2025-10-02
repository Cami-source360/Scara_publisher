import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile
import math
import numpy as np
from icecream import ic


class ScaraInverseKinematics(Node):
    def __init__(self):
        super().__init__('scara_inverse_kinematics')
        
        # ---- Parameters (declare + get) ----
        self.declare_parameter('publish_rate', 50.0)      # Hz
        self.declare_parameter('r1', 0.155)                # Length of first arm (m)
        self.declare_parameter('r2', 0.13)                # Length of second arm (m)
        self.declare_parameter('delta3', 0.015)           # Minimum extension of prismatic joint (m)
        self.declare_parameter('max_prismatic', 0.14)     # Max prismatic extension (m)

        self.rate_hz = float(self.get_parameter('publish_rate').value)
        self.r1 = float(self.get_parameter('r1').value)
        self.r2 = float(self.get_parameter('r2').value)
        self.delta3 = float(self.get_parameter('delta3').value)
        self.max_prismatic = float(self.get_parameter('max_prismatic').value)
        
        # ---- State ----
        # Desired position (in m)
        self.px = 0.0  # m
        self.py = 0.0  # m
        self.pz = 0.0  # m
        
        # Output joint configuration
        self.pj1 = 0.0  # rad
        self.pj2 = 0.0  # rad
        self.pj3 = 0.0  # m
        self.flag = False 

        # ---- Pub/Sub ----
        qos = QoSProfile(depth=10)
        self.create_subscription(Twist, 'twist_mux_out', self._scara_configuration, qos)
        self.publisher_conf = self.create_publisher(Twist, 'inv_kin', qos)
        
        # ---- Timer ----
        self.last_time_conf = None
        self.dt_conf = 1.0/self.rate_hz
        self.timer_conf = self.create_timer(self.dt_conf, self.publish_conf)
        
        self.get_logger().info(f'SCARA Inverse Kinematics node started')
        self.get_logger().info(f'Robot parameters: r1={self.r1}m, r2={self.r2}m, delta3={self.delta3}m')
        self.get_logger().info(f'Prismatic limits: 0m to {self.max_prismatic}m')

    # def _2r_inverse_kinematics(self):
    #     """
    #     Inverse kinematics for a 2R robot manipulator
    #     px, py are the desired end-effector positions (in cm)
    #     th1, th2 are the resulting joint variables (angles in rad)
    #     """
    #     ic(self.px, self.py)
    #     ic(self.l1, self.l2)
         # Check if the position is reachable
        # if d < -1.0 or d > 1.0:
        #     self.get_logger().warn(f'Target position x={self.px:.2f}cm, y={self.py:.2f}cm is out of reach.')
        #     return
    #     r = np.sqrt(np.power(self.px,2) + np.power(self.py,2))
    #     d = (np.power(r,2) - np.power(self.l1,2) - np.power(self.l2,2))/(2*self.l1*self.l2)
        
    #     theta_2 = -np.arctan2(np.sqrt(1 - np.power(d,2)),d)
    #     theta_1 = np.arctan2(self.py,self.px) - np.arctan2(np.sin(theta_2)*self.l2,(self.l1 + np.cos(theta_2)*self.l2))


    #     # now = self.get_clock().now()
    #     # js = JointState()
    #     # js.header.stamp = now.to_msg()
    #     # js.name = ["brazo1_joint", "brazo2_joint", "prismatic_joint"]
    #     # js.position = [theta_1, theta_2, 0.]
    #     # # js.velocity = [w_l, w_r]
    #     # self.pub_js.publish(js)

    #     self.th1 = theta_1
    #     self.th2 = theta_2

   

    def _scara_inverse_kinematics(self):
        """
        Inverse kinematics for a SCARA robot
        px, py, pz are the desired end-effector positions (in m)
        pj1, pj2, pj3 are the resulting joint variables (angles in rad, extension in m)
        """
        ic(self.px, self.py, self.pz)
        ic(self.r1, self.r2)
        
        # Calculate distance to target in xy-plane
        r = np.sqrt(self.px**2 + self.py**2)
        
        # Check workspace limits
        max_reach = self.r1 + self.r2
        min_reach = abs(self.r1 - self.r2)
        
        if r > max_reach:
            self.get_logger().warn(f'Position out of reach: distance={r:.4f}m > max_reach={max_reach:.4f}m')
            return False
        
        if r < min_reach:
            self.get_logger().warn(f'Position too close: distance={r:.4f}m < min_reach={min_reach:.4f}m')
            return False
        
        # Calculate D (cosine of theta2)
        D = (self.px**2 + self.py**2 - self.r1**2 - self.r2**2) / (2 * self.r1 * self.r2)
        D = round(D, 6)
        
        # Ensure value is within valid range for arccos
        if abs(D) > 1.0:
            self.get_logger().warn(f'Position out of reach: D = {D}')
            return False
        
        # Two options for theta2: elbow up and elbow down
        theta2_opt = [math.acos(D), -math.acos(D)]
        theta1_opt = []

        for theta2 in theta2_opt:
            th1 = None
            
            # Check theta2 limits
            if abs(math.degrees(theta2)) > 110:
                self.get_logger().warn(f'Position out of reach: |theta2| > 110 deg ({math.degrees(theta2):.2f} deg)')
                theta1_opt.append(None)
                continue

            # Calculate theta1 using matrix method
            mat = np.array([[self.r1 + self.r2 * math.cos(theta2), -self.r2 * math.sin(theta2)],
                            [self.r2 * math.sin(theta2), self.r1 + self.r2 * math.cos(theta2)]])
            vec = np.array([self.px, self.py])

            if np.linalg.det(mat) == 0:
                self.get_logger().warn('Singular configuration')
                theta1_opt.append(None)
                continue

            mat_inv = np.linalg.inv(mat)
            sol = mat_inv @ vec
            th1 = math.atan2(sol[1], sol[0])

            # Check theta1 limits
            if abs(math.degrees(th1)) > 85:
                self.get_logger().warn(f'Position out of reach: |theta1| > 85 deg ({math.degrees(th1):.2f} deg)')
                th1 = None

            theta1_opt.append(th1)

        # Calculate prismatic joint extension
        prismatic = self.delta3 - self.pz
        
        # Check prismatic limits
        if prismatic < 0.0 or prismatic > self.max_prismatic:
            self.get_logger().warn(f'Prismatic joint out of range: {prismatic:.4f}m (limits: 0.0-{self.max_prismatic:.4f}m)')
            return False

        # Select the first valid (th1, th2) pair
        found_valid = False
        
        for idx, th1 in enumerate(theta1_opt):
            th2 = theta2_opt[idx]
            if th1 is not None and th2 is not None:
                self.pj1 = th1
                self.pj2 = th2
                self.pj3 = prismatic  # Extension of prismatic joint (m)
                found_valid = True
                self.get_logger().debug(f'IK Solution: pj1={math.degrees(th1):.2f}°, pj2={math.degrees(th2):.2f}°, pj3={prismatic:.4f}m')
                break

        if not found_valid:
            self.get_logger().warn('No valid solution for joint angles; keeping previous values.')
            return False
        
        return True

            
    def _scara_configuration(self, msg: Twist):
        """
        Callback for desired position topic
        Input is in meters
        """
        x = msg.linear.x
        y = msg.linear.y
        z = msg.linear.z

        self.px = x / 1000  # End effector position (m)
        self.py = y / 1000  # End effector position (m)
        self.pz = z / 1000  # End effector position (m)
        
        self.get_logger().debug(f'Received desired position: x={self.px:.4f}m, y={self.py:.4f}m, z={self.pz:.4f}m')
        
        if not self.flag:
            self.flag = True
        
    # def _desired_pos_callback(self, msg: Twist):
    #     """
    #     Callback for desired position topic
    #     Converts input from mm to cm for x,y and keeps z in mm
    #     """
    #     # Convert from mm to cm for x and y coordinates
    #     self.px = msg.linear.x * 100.0  # m to cm
    #     self.py = msg.linear.y * 100.0  # m to cm
    #     self.pz = msg.linear.z * 100.0        # keep z in cm
        
    #     self.get_logger().debug(f'Received desired position: x={self.px:.2f}cm, y={self.py:.2f}cm, z={self.pz:.2f}mm')
        
    #     if not self.flag:
    #         self.flag = True

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
            tw.linear.z = self.pz  # Keep z in m

            self.publisher_inv_kin.publish(tw)

            self.get_logger().debug(f'Published joint angles: th1={math.degrees(self.th1):.2f}°, th2={math.degrees(self.th2):.2f}°, z={self.pz:.2f}m')


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