import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
import json
import math

NODES = [
    (-10, -10), (-5, -10), (0, -10), (5, -10), (10, -10),
    (-10, -5),  (-5, -5),  (0, -5),  (5, -5),  (10, -5),
    (-10, 0),   (-5, 0),   (0, 0),   (5, 0),   (10, 0),
    (-10, 5),   (-5, 5),   (0, 5),   (5, 5),   (10, 5),
    (-10, 10),  (-5, 10),  (0, 10),  (5, 10),  (10, 10),
]

DRONES = ["X3_1", "X3_2", "X3_3"]

class CentralPlanner(Node):
    def __init__(self):
        super().__init__('central_planner')
        self.publisher_ = self.create_publisher(String, 'central_plan', 10)
        self.positions = {drone: (0.0, 0.0) for drone in DRONES}
        for drone in DRONES:
            self.create_subscription(Pose, f'{drone}/pose', self.make_pose_callback(drone), 10)
        self.timer = self.create_timer(15.0, self.publish_plan)

    def make_pose_callback(self, drone):
        def callback(msg):
            self.positions[drone] = (msg.position.x, msg.position.y)
        return callback

    def publish_plan(self):
        assigned_nodes = []
        plan = {}
        for drone, (x, y) in self.positions.items():
            closest_node = None
            closest_dist = float('inf')
            for node in NODES:
                if node in assigned_nodes:
                    continue
                dx = node[0] - x
                dy = node[1] - y
                if (dx == 0 and abs(dy) == 5) or (dy == 0 and abs(dx) == 5):
                    dist = math.sqrt(dx**2 + dy**2)
                    if dist < closest_dist:
                        closest_dist = dist
                        closest_node = node
            if closest_node is not None:
                assigned_nodes.append(closest_node)
                plan[drone] = {"x": closest_node[0], "y": closest_node[1]}
        msg = String()
        msg.data = json.dumps(plan)
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published new plan: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    central_planner = CentralPlanner()
    rclpy.spin(central_planner)
    central_planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
