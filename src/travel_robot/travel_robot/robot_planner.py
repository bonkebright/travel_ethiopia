import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from collections import deque


# ---------------- GRAPH (Adjacency List) ---------------- #

ADJ = {

    "Gambella": ["Dembi Dollo"],
    "Dembi Dollo": ["Gambella", "Gore"],
    "Gore": ["Dembi Dollo", "Mezan Teferi", "Bedelle"],
    "Mezan Teferi": ["Gore", "Tepi"],
    "Tepi": ["Mezan Teferi", "Bonga"],
    "Bonga": ["Tepi", "Jimma"],
    "Jimma": ["Bonga", "Wolkite", "Ambo"],
    "Bedelle": ["Gore", "Nekemte"],
    "Nekemte": ["Bedelle", "Ambo"],
    "Ambo": ["Nekemte", "Addis Ababa", "Jimma"],
    "Wolkite": ["Jimma", "Worabe"],
    "Worabe": ["Wolkite", "Hossana", "Buta Jirra"],
    "Hossana": ["Worabe", "Wolaita Sodo"],
    "Wolaita Sodo": ["Hossana", "Arba Minch"],
    "Arba Minch": ["Wolaita Sodo", "Konso"],
    "Konso": ["Arba Minch", "Bule Hora"],
    "Bule Hora": ["Konso", "Yabello"],
    "Yabello": ["Bule Hora", "Moyale"],
    "Moyale": ["Yabello"],

    "Addis Ababa": ["Ambo", "Debre Birhan", "Buta Jirra", "Adama"],
    "Debre Birhan": ["Addis Ababa"],
    "Buta Jirra": ["Addis Ababa", "Batu"],
    "Batu": ["Buta Jirra", "Shashemene"],
    "Shashemene": ["Batu", "Hawassa", "Dodolla"],
    "Hawassa": ["Shashemene", "Dilla"],
    "Dilla": ["Hawassa"],
    "Dodolla": ["Shashemene", "Assasa"],
    "Assasa": ["Dodolla", "Assella"],
    "Assella": ["Assasa", "Adama"],
    "Adama": ["Addis Ababa", "Assella", "Matahara"],
    "Matahara": ["Adama", "Awash"],
    "Awash": ["Matahara", "Chiro"],
    "Chiro": ["Awash", "Dire Dawa"],
    "Dire Dawa": ["Chiro", "Harar"],
    "Harar": ["Dire Dawa", "Babile"],
    "Babile": ["Harar", "Jigjiga"],
    "Jigjiga": ["Babile", "Dega Habur"],
    "Dega Habur": ["Jigjiga", "Kebri Dehar"],
    "Kebri Dehar": ["Dega Habur", "Gode"],
    "Gode": ["Kebri Dehar"],
}


# ---------------- CITY POSITIONS (Gazebo meters) ---------------- #

POS = {
  "Gambella":     [0,   3],
  "Dembi Dollo":  [1,   3],
  "Gore":         [2,   3.5],
  "Mezan Teferi": [2,   5],
  "Gimbi":        [2,   2.5],
  "Tepi":         [3,   4.5],
  "Bedelle":      [3,   3],
  "Nekemte":      [3,   2],
  "Bonga":        [4,   5],
  "Jimma":        [4,   4],
  "Wolkite":      [4.5, 3],
  "Dawro":        [4.5, 5.5],
  "Ambo":         [4,   2],
  "Worabe":       [5,   3.5],
  "Hossana":      [5,   4.5],
  "Wolaita Sodo": [5,   5.5],
  "Addis Ababa":  [6,   2],
  "Buta Jirra":   [6,   3],
  "Batu":         [6.5, 3.5],
  "Arba Minch":   [5.5, 6.5],
  "Debre Birhan": [7,   1],
  "Adama":        [7.5, 2.5],
  "Shashemene":   [7,   4.5],
  "Hawassa":      [6.5, 5.5],
  "Dilla":        [6,   6.5],
  "Konso":        [5.5, 7.5],
  "Bule Hora":    [6,   7.5],
  "Assella":      [8,   3],
  "Matahara":     [8.5, 2],
  "Assasa":       [8,   4.5],
  "Dodolla":      [8,   5.5],
  "Yabello":      [6,   8.5],
  "Awash":        [9.5, 2],
  "Bale":         [9,   6.5],
  "Moyale":       [6,   10],
  "Chiro":        [10.5,2],
  "Sof Oumer":    [10,  7],
  "Goba":         [10,  6],
  "Liben":        [9,   8],
  "Dire Dawa":    [11.5,2],
  "Gode":         [11,  7.5],
  "Harar":        [12,  3],
  "Dega Habur":   [12,  5],
  "Babile":       [12,  4],
  "Kebri Dehar":  [12,  6.5],
  "Jigjiga":      [13,  4.5],
  "Werder":       [13,  7],
}


# ---------------- ROS2 Planner Node ---------------- #

class RobotPlanner(Node):

    def __init__(self):
        super().__init__('robot_planner')

        self.path_pub = self.create_publisher(Path, '/planned_path', 10)

        self.timer = self.create_timer(2.0, self.plan_and_publish)

        self.start_city = "Addis Ababa"
        self.goal_city = "Harar"

        self.get_logger().info("Robot Planner Started")

    # -------- BFS Search -------- #

    def bfs(self, start, goal):

        queue = deque([[start]])
        visited = set()

        while queue:
            path = queue.popleft()
            city = path[-1]

            if city == goal:
                return path

            if city not in visited:
                visited.add(city)

                for neighbor in ADJ.get(city, []):
                    new_path = list(path)
                    new_path.append(neighbor)
                    queue.append(new_path)

        return []

    # -------- Plan & Publish -------- #

    def plan_and_publish(self):

        path = self.bfs(self.start_city, self.goal_city)

        if not path:
            self.get_logger().error("No path found!")
            return

        self.get_logger().info(f"Path found: {path}")

        msg = Path()
        msg.header.frame_id = "map"

        for city in path:

            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = self.get_clock().now().to_msg()

            pose.pose.position.x = POS[city][0]
            pose.pose.position.y = POS[city][1]
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0

            msg.poses.append(pose)

        self.path_pub.publish(msg)


def main():
    rclpy.init()
    node = RobotPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
