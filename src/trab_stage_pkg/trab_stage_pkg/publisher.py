#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

class Publisher(Node):
    def __init__(self):
        super().__init__("Publisher")
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.timer = self.create_timer(0.5, self.publisher_hello)
        self.get_logger().info("Publisher started")

    def publisher_hello(self):
        msg = Twist() # so move em x, z
        msg.linear.x = 0.2
        #msg.angular.z = 0.2
        self.publisher.publish(msg)

def main():
    rclpy.init(args=None)
    node=Publisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__== "__main__":
    main()