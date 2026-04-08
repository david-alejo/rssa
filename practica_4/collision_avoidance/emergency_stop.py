#!/usr/bin/python3

# A very basic Emergency Stop script

import math

import py
import rclpy
import tf2_ros
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan

class EmergencyStop(Node):
    def __init__(self): 
        super().__init__('emergency_stop')
	    # Create a publisher which can "talk" to TurtleBot and tell it to move
        # Tip: You may need to change /cmd_vel to the proper velocity command topic
        self.cmd_vel_pub = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        self.cmd_vel_sub = self.create_subscription(TwistStamped, 'v_pref', self.vPrefCallback, 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scanCallback, 10)
        
        # Create a timer so that the control node is executed periodically each 0.1 seconds
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.v_pref = None
        self.min_d = 0.25
        self.emergency_stop = True
        
        print("Emergency stop node initialized.")
        
    def control_loop(self):
        if self.v_pref is None:
            self.get_logger().debug("No velocity command received yet.")
            return
        
        ### TODO: implement the control loop. Hint: if emergency stop is active, set the linear velocity to 0, but not the angular velocity (so that the robot can rotate to avoid the obstacle)
        if self.emergency_stop:
            print("Stopping the robot")
        print("Publishing velocity command: linear.x = %.2f, angular.z = %.2f".format(self.v_pref.twist.linear.x, self.v_pref.twist.angular.z))
        self.cmd_vel_pub.publish(self.v_pref)
        ## End of task
        
        print("Published")


    def vPrefCallback(self, msg):
        # Hint: If no path is received it is set to None in the constructor (path received) 
        # And also reset the current wp counter (you can set it to the closest point to the robot)
        self.v_pref = msg

# Exercise 1: implement scan callback
    def scanCallback(self, msg):
        self.get_logger().debug("Scan received. Number of points: %d", len(msg.ranges))
        #TODO: Define the actions that should be carried out whenever a path is received
        # Hint: If no path is received it is set to None in the constructor (path received) 
        # And also reset the current wp counter (you can set it to the closest point to the robot)
        self.emergency_stop = False
        
        print("Here")
        
        angle = msg.angle_min
        for distance in msg.ranges:
            #### Move to range -PI, PI
            if angle > math.pi:
                angle -= 2*math.pi
            
            # TODO: see if the ranges of interest are in the front of the robot, and if any of them is below the threshold, activate the emergency stop 
             
            ## End of task
            
            angle+=msg.angle_increment
# end of Exercise 1


def main(args=None):
        rclpy.init(args=args)
        #try:
	    # initiliaze
	    # tell user how to stop TurtleBot
        print("Emergency stop node. It will stop the robot in case of an imminent collision.")

        node = EmergencyStop()
	    # What function to call when you ctrl + c    

	    
	    # as long as you haven't ctrl + c keeping doing...
        rclpy.spin(node)

 
if __name__ == '__main__':
    main()
    
    
