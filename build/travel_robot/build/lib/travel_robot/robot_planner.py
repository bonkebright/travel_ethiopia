#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from collections import deque


# ============================================================
# State Space Graph (Uninformed Search Graph)
# ============================================================

ADJACENCY = {
    "Addis Ababa": ["Ambo", "Adama", "Debre Birhan"],
    "Ambo": ["Addis Ababa"],
    "Adama": ["Addis Ababa"],
    "Debre Birhan": ["Addis Ababa"],
}


# ============================================================
# BFS Algorithm
# ============================================================

def bfs(graph, start, goal):

    if start not in graph or goal not in graph:
        return None

    queue = deque([[start]])
    visited = {start}

    while queue:
        path = queue.popleft()
        current = path[-1]

        if current == goal:
            return path

        for neighbor in graph.get(current, []):
            if neighbor not in visited:
                visited.add(neighbor)
                queue.append(path + [neighbor])

    return None


# ============================================================
# ROS2 Planner Node
# ============================================================

class RobotPlanner(Node):

    def __init__(self):
        super().__init__("robot_planner")

        # Parameters
        self.declare_parameter("start", "Addis Ababa")
        self.declare_parameter("goal", "Adama")

        self.start = self.get_parameter("start").value
        self.goal = self.get_parameter("goal").value

        # Publisher
        self.path_publisher = self.create_publisher(String, "planned_path", 10)

        self.get_logger().info(
            f"Planner Started | Start: {self.start} | Goal: {self.goal}"
        )

        # Run search once after startup
        self.timer = self.create_timer(1.0, self.run_search)
        self.executed = False

    # --------------------------------------------------------

    def run_search(self):

        if self.executed:
            return

        self.executed = True

        self.get_logger().info("Running BFS Search...")

        path = bfs(ADJACENCY, self.start, self.goal)

        msg = String()

        if path:
            path_str = " -> ".join(path)

            self.get_logger().info(f"Path Found: {path_str}")

            msg.data = path_str

        else:
            self.get_logger().warn("No Path Found")
            msg.data = ""

        self.path_publisher.publish(msg)


# ============================================================

def main():
    rclpy.init()
    node = RobotPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
