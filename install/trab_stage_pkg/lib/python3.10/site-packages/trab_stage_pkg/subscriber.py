#!/usr/bin/env python3
import rclpy
import math
import numpy as np

from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from tf_transformations import euler_from_quaternion 

def estimated_Pose(odom_pos, odom_ori):
    pose_odom = np.array([odom_pos.x, odom_pos.y, 1.0])

    theta = self.offset_yaw  
    cos_t = math.cos(theta)
    sin_t = math.sin(theta)

    T_odom_to_world = np.array([
        [cos_t, -sin_t, self.offset_x],
        [sin_t,  cos_t, self.offset_y],
        [0.0,    0.0,   1.0]
    ])

    pose_world = T_odom_to_world @ pose_odom

    world_x = pose_world[0]
    world_y = pose_world[1]

    quat = [odom_ori.x, odom_ori.y, odom_ori.z, odom_ori.w]
    _, _, yaw = euler_from_quaternion(quat)

    world_yaw = yaw + self.offset_yaw
    world_yaw = (world_yaw + math.pi) % (2 * math.pi) - math.pi

    return world_x, world_y, world_yaw

class Subscriber(Node):
    def __init__(self):
        super().__init__("subscriber")

        # Assinantes
        self.create_subscription(LaserScan, "/base_scan", self.callback_scan, 10)
        self.create_subscription(Odometry, "/odom", self.callback_odom, 10)
        
        self.timer = self.create_timer(0.5, self.publisher_hello)

        self.offset_x = -7.0
        self.offset_y = -7.0

        self.offset_yaw = math.radians(45.0)

        self.get_logger().info("Subscriber com transformação manual iniciado.")

    def callback_scan(self, msg):
        self.get_logger().info(f'Recebido LaserScan com {len(msg.ranges)} pontos')
        self.get_logger().info(f'Primeiro valor de range: {msg.ranges[0]} metros')

    def callback_odom(self, msg):
        odom_pos = msg.pose.pose.position
        odom_ori = msg.pose.pose.orientation

        world_x, world_y, world_yaw = estimated_Pose(odom_pos, odom_ori)
        
        self.get_logger().info(
            f"Pose em relação ao mundo: x={world_x:.2f}, y={world_y:.2f}, yaw={math.degrees(world_yaw):.2f}°"
        )

def main():
    rclpy.init()
    node = Subscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == "__main__":
    main()
