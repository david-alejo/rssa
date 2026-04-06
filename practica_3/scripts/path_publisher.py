#!/usr/bin/python3
# -*- coding: utf-8 -*-

import sys
import math
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import yaml

class PathPublisher(Node):
    def __init__(self):
        super().__init__('path_publisher')
        self.publisher = self.create_publisher(Path, '/path', 10)
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('path_file', rclpy.Parameter.Type.STRING),
                ('frame', rclpy.Parameter.Type.STRING),
            ]   
        )
        
        with open(self.get_parameter('path_file').value, 'r') as f:
            path_data = yaml.safe_load(f)
        
        self.goals = path_data['path']
        self.timer = self.create_timer(1.0, self.publish_path)

    def publish_path(self):
        path = Path()
        path.header.frame_id = self.get_parameter('frame').value or "map"
        path.header.stamp = self.get_clock().now().to_msg()
        
        for i in self.goals:
            pose = PoseStamped()
            pose.header.frame_id = path.header.frame_id
            print(i)
            pose.pose.position.x = self.goals[i]['x'] 
            pose.pose.position.y = self.goals[i]['x'] 
            pose.pose.position.z = 0.0
            
            path.poses.append(pose)
            self.get_logger().info(f"Added point {pose.pose.position.x}, {pose.pose.position.y}")
        
        self.publisher.publish(path)
        self.get_logger().info("Path published!")

def main(args=None):
    rclpy.init(args=args)
    path_publisher = PathPublisher()
    rclpy.spin(path_publisher)
    path_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()