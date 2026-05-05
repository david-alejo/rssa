#!/usr/bin/python3

import sys
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path    
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from dijkstra import Dijkstra
from tf2_ros import TransformListener, Buffer, TransformException

class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner_node')
        
        # Create a timer so that the control node is executed periodically each 0.1 seconds
        
        
        # Create a publisher which can "talk" to TurtleBot and tell it to move
        self.path_pub = self.create_publisher(Path, 'path', 10)
        self.goal_sub = self.create_subscription(PoseStamped, 'goal_pose', self.goalCallback, 10)
        self.map_sub = self.create_subscription(OccupancyGrid, 'costmap', self.mapCallback, qos_profile=rclpy.qos.QoSProfile(depth=1, durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL))
        
        # Parameters
        self.declare_parameter('base_frame_id', 'base_footprint')
        self.declare_parameter('global_frame_id', 'map')
        self.declare_parameter('min_obstacle_value', 70)  # Valor mínimo para considerar una celda como obstáculo en el mapa de ocupación
        
        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.global_frame_id = self.get_parameter('global_frame_id').value
        self.min_obstacle_value = self.get_parameter('min_obstacle_value').value
        
        self.goal_received = False
        self.goal = None
        
        self.map = None

        # Transform listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
    
        self.k_angular = 1.0
        
    def control_loop(self):
        if self.goal_received:
            self.path_pub.publish(self.path)

    def goalCallback(self, msg):
        self.get_logger().info("Path Planner Node: Goal received: " + str(msg.pose.position.x) + ", " + str(msg.pose.position.y))
        self.goal = msg
        

        if self.map is not None:
            self.goal_received = True
            self.path = self.compute_path(self.goal)

    def mapCallback(self, msg):
        self.map = msg
        self.get_logger().info("Map received")
        self.dijkstra = Dijkstra(self.map)
        
    def compute_path(self, goal):
        transform = None
        try:
            transform = self.tf_buffer.lookup_transform(self.global_frame_id, self.base_frame_id, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=3.0))
        except TransformException as ex:
            self.get_logger().info(f'Path planner: could not transform goal')
            return

        start_x = transform.transform.translation.x
        start_y = transform.transform.translation.y
        goal_x = goal.pose.position.x
        goal_y = goal.pose.position.y
        path_x, path_y = self.dijkstra.plan(start_x, start_y, goal_x, goal_y)

        self.get_logger().info("Start: " + str(start_x) + ", " + str(start_y))
        self.get_logger().info("Goal: " + str(goal_x) + ", " + str(goal_y))

        path_msg = Path()
        path_msg.header.frame_id = self.global_frame_id
        path_msg.header.stamp = self.get_clock().now().to_msg()
        for x, y in zip(path_x, path_y):
            pose = PoseStamped()
            pose.header.frame_id = self.global_frame_id
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.get_logger().info("Path computed with " + str(len(path_msg.poses)) + " poses.")

        self.path_pub.publish(path_msg)
        return path_msg


def main(args=None):
    rclpy.init(args=args)
    robot = PathPlanner()
    robot.get_logger().info("Initializing Path Planner Node. Using Dijkstra algorithm.")
    try:
        rclpy.spin(robot)

    except Exception as e:
        robot.get_logger().info(f"Path Planner Node terminated: {e}")

if __name__ == '__main__':
    main()
