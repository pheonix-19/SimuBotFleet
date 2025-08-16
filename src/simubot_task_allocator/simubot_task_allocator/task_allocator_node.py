
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from typing import List, Dict, Tuple
import math
import json
import os

def dist(a: Tuple[float,float], b: Tuple[float,float]) -> float:
    return math.hypot(a[0]-b[0], a[1]-b[1])

class TaskAllocator(Node):
    """
    Minimal policy:
      - Robots: robot_1..robot_N
      - Subscribe: '/fleet/tasks' (std_msgs/String with JSON: {"pick":[x,y], "drop":[x,y], "priority":1})
      - Publish: Pose goals to '/robot_i/goal_pose' (Nav2-accepted topic remapped by bringup)
    Replace policy with RL later *without* breaking interfaces.
    """
    def __init__(self):
        super().__init__('task_allocator')
        self.declare_parameter('robot_count', 2)
        self.robot_count = int(self.get_parameter('robot_count').value)

        # Track "fake" robot poses (could be replaced by TF lookup or odom subscription)
        self.robot_xy: Dict[str, Tuple[float,float]] = {f'robot_{i+1}': (i*1.5, 0.0) for i in range(self.robot_count)}
        self.publishers = {
            name: self.create_publisher(PoseStamped, f'/{name}/goal_pose', 10)
            for name in self.robot_xy.keys()
        }

        self.task_sub = self.create_subscription(String, '/fleet/tasks', self.on_task, 10)
        self.get_logger().info(f'TaskAllocator ready for {self.robot_count} robots.')

    def on_task(self, msg: String):
        try:
            task = json.loads(msg.data)
            pick = tuple(task['pick'])
            drop = tuple(task['drop'])
        except Exception as e:
            self.get_logger().error(f'Bad task payload: {e}')
            return

        # Pick the nearest robot to "pick"
        best_robot = None
        best_d = 1e9
        for name, xy in self.robot_xy.items():
            d = dist(xy, pick)
            if d < best_d:
                best_d = d
                best_robot = name

        if not best_robot:
            self.get_logger().warn('No robot available.')
            return

        self.get_logger().info(f'Assigning task to {best_robot}: pick {pick} -> drop {drop}')

        # Send Nav2 goal sequence: pick first, then drop (simple demo: publish pick now)
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = float(pick[0])
        goal.pose.position.y = float(pick[1])
        goal.pose.orientation.w = 1.0

        self.publishers[best_robot].publish(goal)

    # (Extend later with battery, workload, RL, etc. without interface change.)

def main():
    rclpy.init()
    node = TaskAllocator()
    rclpy.spin(node)
    rclpy.shutdown()
