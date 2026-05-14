#!/usr/bin/python3

import sys
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformListener, Buffer

class TurtlebotController(Node):
    def __init__(self):
        super().__init__('turtlebot_controller')
        
        # Create a timer so that the control node is executed periodically each 0.1 seconds
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # Create a publisher which can "talk" to TurtleBot and tell it to move
        self.cmd_vel_pub = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        
        # Create a Transform Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Parameters
        self.base_frame_id = "base_footprint"
        self.global_frame_id = "odom"
        self.v_ref = 0.5
        
        self.goal_received = False
        self.goal = PoseStamped()
        self.goal.pose.orientation.w = 1.0
        self.goal.header.frame_id = "odom"
        self.goal.pose.position.x = 2.0
        self.goal.pose.position.y = 2.0
        self.goal.pose.position.z = 0.0
        
    def control_loop(self):
        self.get_logger().info("Turtlebot Controller: Control Loop")
        
        linear = 0.0
        angular = 0.0

        base_goal = PoseStamped()
        
        # Implement control logic here
        
        self.publish(linear, angular)

    def publish(self, lin_vel, ang_vel):
        move_cmd = TwistStamped()
        move_cmd.header.stamp = self.get_clock().now().to_msg()
        move_cmd.twist.linear.x = lin_vel
        move_cmd.twist.angular.z = ang_vel
        self.cmd_vel_pub.publish(move_cmd)
        
    def shutdown(self):
        #self.get_logger().info("Stop TurtleBot")
        #self.cmd_vel_pub.publish(TwistStamped())
        print("Turtlebot Controller: Stopping TurtleBot")
        

def main(args=None):
    rclpy.init(args=args)
    robot = TurtlebotController()
    robot.get_logger().info("Initializing Turtlebot Controller. Please press CTRL + C to stop TurtleBot ")
    try:
        rclpy.spin(robot)

    except Exception as e:
        robot.get_logger().info(f"Turtlebot_controller node terminated: {e}")
    finally:
        robot.shutdown()

if __name__ == '__main__':
    main()
