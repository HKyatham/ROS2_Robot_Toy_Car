#!/usr/bin/env python3
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy import logging
import math
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Imu

class Controller_Node(Node):
    def __init__(self):
        super().__init__('controller_node')
        logging.get_logger(name="controller_node").info('Node Started')
        

        self.desired_x = 10.0  # desired position 
        self.desired_y = 10.0  # desired position
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
            )
      

        # Publisher and Subscriber
        self.my_pose_sub = self.create_subscription(Imu, "/imu_plugin/out", self.pose_callback, qos_profile)
        self.joint_position_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.wheel_velocities_pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)

    def pose_callback(self, msg: Imu):
        logging.get_logger(name="controller_node").info(f"Current x={msg.orientation.x} current y={msg.orientation.z} and current angle = {msg.orientation.w}")
        # Calculate errors in position
        err_x = self.desired_x - msg.orientation.x
        err_y = self.desired_y - msg.orientation.z
        err_dist = (err_x**2+err_y**2)**0.5
        
        # Distance error (magnitude of the error vector)
        
        logging.get_logger(name="controller_node").info(f"Error in x {err_x} and error in y {err_y}")

        # Desired heading based on the position error
        desired_theta = math.atan2(err_y, err_x)
        
        # Error in heading
        err_theta = desired_theta - msg.orientation.w
       
        # Handle wrap-around issues (e.g., if error jumps from +pi to -pi)
        while err_theta > math.pi:
            err_theta -= 2.0 * math.pi
        while err_theta < -math.pi:
            err_theta += 2.0 * math.pi
        logging.get_logger(name="controller_node").info(f"Desired Angle = {desired_theta} current angle {msg.orientation.w} Error angle {err_theta}")
        # constant for linear velocity (distance control)

        Kp_dist = 0.4
            


        # constants for angular velocity (heading control)
        Kp_theta = 2
        ki=0.1
        kd=0.05


        # PID control for linear velocity
        #l_v = Kp_dist * abs(err_x) # + Ki_dist * integral_dist + Kd_dist * derivative_dist
        l_v = Kp_dist * abs(err_dist) + err_dist * ki + kd * err_dist


        # PID control for angular velocity
        a_v = Kp_theta * err_theta  

        # Send the velocities
        self.my_velocity_cont(l_v, a_v)

    def my_velocity_cont(self, l_v, a_v):
        logging.get_logger(name="controller_node").info(f"Commanding liner ={l_v} and angular ={a_v}")
        joint_positions = Float64MultiArray()
        wheel_velocities = Float64MultiArray()
        wheel_velocities.data = [-l_v,l_v]
        joint_positions.data = [a_v]
        self.joint_position_pub.publish(joint_positions)
        self.wheel_velocities_pub.publish(wheel_velocities)
    
    def get_logger():
        return rclpy

def main(args=None):
    rclpy.init(args=args)
    node = Controller_Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()