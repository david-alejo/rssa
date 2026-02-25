#!/usr/bin/python3
# -*- coding: utf-8 -*-

import sys
import math
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

class PathPublisher(Node):
    def __init__(self):
        super().__init__('path_publisher')
        self.publisher = self.create_publisher(Path, '/path', 10)
        self.seq = 0
        self.goals = sorted(self.get_parameter('path').value.items())
        self.timer = self.create_timer(1.0, self.publish_path)

    def publish_path(self):
        path = Path()
        path.header.frame_id = self.get_parameter('frame').value or "map"
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.seq = self.seq
        seq_goals = 0
        
        for i in self.goals:
            pose = PoseStamped()
            pose.header.frame_id = path.header.frame_id
            pose.header.seq = seq_goals
            seq_goals += 1
            pose.pose.position.x = i[1]['x']
            pose.pose.position.y = i[1]['y']
            pose.pose.position.z = 0
            
            path.poses.append(pose)
            self.get_logger().info(f"Added point {pose.pose.position.x}, {pose.pose.position.y}")
        
        self.publisher.publish(path)
        self.get_logger().info("Path published!")
        self.seq += 1

def main(args=None):
    rclpy.init(args=args)
    path_publisher = PathPublisher()
    rclpy.spin(path_publisher)
    path_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()