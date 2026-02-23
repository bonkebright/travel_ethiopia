#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import time


# Coordinates from your world
COORDINATES = {
    "Addis Ababa": (0, 0),
    "Adama": (1, -1),
}


class PathFollower(Node):

    def __init__(self):
        super().__init__("path_follower")

        self.subscription = self.create_subscription(
            String,
            "planned_path",
            self.path_callback,
            10
        )

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.get_logger().info("Path Follower Started 🚀")

    # -----------------------------------------------------

    def path_callback(self, msg):

        if not msg.data:
            self.get_logger().warn("No path received.")
            return

        cities = msg.data.split(" -> ")

        self.get_logger().info(f"Following Path: {cities}")

        for city in cities:
            if city in COORDINATES:
                self.move_to_city(city)

        self.stop_robot()

    # -----------------------------------------------------

    def move_to_city(self, city):

        self.get_logger().info(f"Moving toward {city}")

        twist = Twist()
        twist.linear.x = 0.2

        # Move forward for 5 seconds (simple demo motion)
        start = time.time()

        while time.time() - start < 5.0:
            self.cmd_pub.publish(twist)
            time.sleep(0.1)

    # -----------------------------------------------------

    def stop_robot(self):

        self.get_logger().info("Stopping Robot 🛑")

        twist = Twist()
        self.cmd_pub.publish(twist)


# ======================================================

def main():
    rclpy.init()
    node = PathFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
