#!/usr/bin/env python3

import rclpy
import math
import numpy as np

from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from tf_transformations import euler_from_quaternion

def estimated_Pose(odom_pos, odom_ori, offset):
    pose_odom = np.array([odom_pos.x, odom_pos.y, 1.0])
    offset_x, offset_y, offset_yaw = offset
    cos_t = math.cos(offset_yaw)
    sin_t = math.sin(offset_yaw)
    T_odom_to_world = np.array([
        [cos_t, -sin_t, offset_x],
        [sin_t,  cos_t, offset_y],
        [0.0,    0.0,   1.0]
    ])
    pose_world = T_odom_to_world @ pose_odom
    world_x, world_y = pose_world[:2]
    quat = [odom_ori.x, odom_ori.y, odom_ori.z, odom_ori.w]
    _, _, yaw = euler_from_quaternion(quat)
    world_yaw = yaw + offset_yaw
    world_yaw = (world_yaw + math.pi) % (2 * math.pi) - math.pi
    return world_x, world_y, world_yaw

def control(x, y, theta, x_d, y_d, theta_d):
    k_rho = 0.3
    k_alpha = 0.8
    k_beta = -0.25

    dx = x_d - x
    dy = y_d - y
    rho = math.hypot(dx, dy)
    alpha = math.atan2(dy, dx) - theta
    alpha = math.atan2(math.sin(alpha), math.cos(alpha))

    if rho > 0.15:
        v = k_rho * rho
        omega = k_alpha * alpha
    else:
        v = 0.0
        beta = theta_d - theta
        beta = math.atan2(math.sin(beta), math.cos(beta))
        omega = k_beta * beta * -1

    return v, omega, rho, abs(theta_d-theta), alpha

class Robot(Node):
    def __init__(self):
        super().__init__("robot_stable_follower_v2")
        self.cmd_vel_publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        
        self.create_subscription(Odometry, "/odom", self._callback_odom, 10)
        self.create_subscription(LaserScan, "/base_scan", self._callback_scan, 10)

        self.timer = self.create_timer(0.1, self._publish_cmd_vel)

        self.offset = np.array([-7.0, -7.0, math.radians(45.0)])

        self.targets = [
            (7.0, -3.0, math.radians(180.0)),
            (7.0, 7.0, math.radians(0.0)),
        ]
        self.current_target_index = 0

        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.lidar_msg = None

        self.state = 'GO_TO_GOAL'
        self.obstacle_detection_distance = 0.8
        self.wall_follow_distance = 0.7
        self.front_cone_angle = math.radians(180.0)
        self.path_check_cone_angle = math.radians(10.0)
        self.max_avoid_speed = 0.22
        self.turn_speed = 0.6
        self.p_gain = 1.5
        self.kd_gain = 2.5
        self.previous_wall_error = 0.0

        self.get_logger().info("Robô Iniciado")

    def set_cmd_vel(self, linear=None, angular=None):
        self.linear_vel = linear if linear is not None else self.linear_vel
        self.angular_vel = angular if angular is not None else self.angular_vel

    def _publish_cmd_vel(self):
        msg = Twist()
        msg.linear.x = self.linear_vel
        msg.angular.z = self.angular_vel
        self.cmd_vel_publisher.publish(msg)

    def _callback_scan(self, msg):
        self.lidar_msg = msg

    def check_obstacle_in_path(self, angle_to_target):
        if self.lidar_msg is None: return False
        
        cone_half_angle = self.path_check_cone_angle / 2.0
        num_ranges = len(self.lidar_msg.ranges)
        center_index = int((angle_to_target - self.lidar_msg.angle_min) / self.lidar_msg.angle_increment)
        cone_offset = int(cone_half_angle / self.lidar_msg.angle_increment)
        start_index = max(0, center_index - cone_offset)
        end_index = min(num_ranges - 1, center_index + cone_offset)
        path_ranges = [r for r in self.lidar_msg.ranges[start_index:end_index+1] if not math.isnan(r) and not math.isinf(r)]
        
        if not path_ranges: return False
        min_dist_in_path = min(path_ranges)
        
        return min_dist_in_path < self.obstacle_detection_distance

    def wall_follower_strategy(self):
        if self.lidar_msg is None: return 0.0, 0.0

        ranges = self.lidar_msg.ranges
        num_ranges = len(ranges)
        
        angle_right = -math.pi / 2.0
        right_idx = int((angle_right - self.lidar_msg.angle_min) / self.lidar_msg.angle_increment)
        right_idx = max(0, min(num_ranges - 1, right_idx))
        dist_right = ranges[right_idx] if not math.isnan(ranges[right_idx]) else float('inf')

        cone_half_angle = self.front_cone_angle / 2.0
        center_idx = num_ranges // 2
        cone_offset = int(cone_half_angle / self.lidar_msg.angle_increment)
        start_idx = max(0, center_idx - cone_offset)
        end_idx = min(num_ranges - 1, center_idx + cone_offset)
        
        front_cone_ranges = [r for r in ranges[start_idx:end_idx+1] if not math.isnan(r)]
        min_dist_front = min(front_cone_ranges) if front_cone_ranges else float('inf')

        if min_dist_front < self.obstacle_detection_distance:
            self.get_logger().warn(f"Obstáculo a {min_dist_front:.2f}m.")
            self.previous_wall_error = 0.0
            v = 0.0
            omega = self.turn_speed
            return v, omega

        if dist_right < self.wall_follow_distance * 3.0:
            v = self.max_avoid_speed
            
            error = self.wall_follow_distance - dist_right
            derivative = error - self.previous_wall_error
            
          
            omega = (self.p_gain * error + self.kd_gain * derivative)
            
            self.previous_wall_error = error
            
            omega = max(-self.turn_speed, min(self.turn_speed, omega))
            return v, omega
            
        self.previous_wall_error = 0.0
        v = self.max_avoid_speed * 0.3
        omega = -self.turn_speed 
        return v, omega

    def _callback_odom(self, msg):
        if self.state == "DONE" or self.lidar_msg is None:
            self.set_cmd_vel(0.0, 0.0)
            return

        world_x, world_y, world_yaw = estimated_Pose(msg.pose.pose.position, msg.pose.pose.orientation, self.offset)
        
        self.get_logger().info(f"Estimated Pose: x:{world_x} | y:{world_y} | rz:{world_yaw}")

        if self.current_target_index >= len(self.targets):
            self.state = "DONE"
            return
            
        x_d, y_d, theta_d = self.targets[self.current_target_index]
        _, _, rho, beta, alpha = control(world_x, world_y, world_yaw, x_d, y_d, theta_d)

        if self.state == 'GO_TO_GOAL':
            if self.check_obstacle_in_path(alpha):
                self.state = 'AVOID_OBSTACLE'
                self.previous_wall_error = 0.0
                self.get_logger().info(">>>> MUDANDO PARA ESTADO: AVOID_OBSTACLE")
            else:
                v, omega, _, _, _ = control(world_x, world_y, world_yaw, x_d, y_d, theta_d)
                self.set_cmd_vel(v, omega)
                self.get_logger().info(f"Estado: GO_TO_GOAL | Alvo {self.current_target_index+1} | Dist: {rho:.2f} m")
                if rho < 0.15 and beta < math.radians(5.0):
                    self.current_target_index += 1
                    if self.current_target_index >= len(self.targets):
                        self.set_cmd_vel(0.0, 0.0)
                        self.get_logger().info("TODOS OS ALVOS FORAM ALCANÇADOS.")
                        self.state = "DONE"
                    else:
                        self.get_logger().info(f"ALVO ALCANÇADO! Próximo alvo: {self.current_target_index+1}")


        elif self.state == 'AVOID_OBSTACLE':
            if abs(alpha) < math.radians(20) and not self.check_obstacle_in_path(alpha):
                 self.state = 'GO_TO_GOAL'
                 self.get_logger().info("<<<< CAMINHO LIVRE! RETORNANDO PARA ESTADO: GO_TO_GOAL")
            else:
                v, omega = self.wall_follower_strategy()
                self.set_cmd_vel(v, omega)

def main():
    rclpy.init()
    node = Robot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.set_cmd_vel(0.0, 0.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()