import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist
import math


class PathFollower(Node):

    def __init__(self):
        super().__init__('path_follower')

        self.path_sub = self.create_subscription(
            Path,
            '/planned_path',
            self.path_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.current_pose = None
        self.path = []
        self.index = 0

        self.timer = self.create_timer(0.1, self.control_loop)

    # ---------------- Receive Path ---------------- #
    def path_callback(self, msg):
        self.get_logger().info("Path received!")
        self.path = msg.poses
        self.index = 0

    # ---------------- Receive Odometry ---------------- #
    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    # ---------------- Control Loop ---------------- #
    def control_loop(self):

        if not self.path or self.current_pose is None:
            return

        if self.index >= len(self.path):
            self.stop_robot()
            return

        target = self.path[self.index].pose.position
        current = self.current_pose.position

        dx = target.x - current.x
        dy = target.y - current.y
        distance = math.sqrt(dx*dx + dy*dy)

        if distance < 0.2:
            self.get_logger().info("Reached waypoint")
            self.index += 1
            return

        angle_to_target = math.atan2(dy, dx)

        twist = Twist()
        twist.linear.x = 0.5 * distance
        twist.angular.z = 1.5 * angle_to_target

        self.cmd_pub.publish(twist)

    def stop_robot(self):
        twist = Twist()
        self.cmd_pub.publish(twist)


def main():
    rclpy.init()
    node = PathFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
