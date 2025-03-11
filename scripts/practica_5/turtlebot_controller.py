#!/usr/bin/python3

# A very basic TurtleBot script that moves TurtleBot forward indefinitely. Press CTRL + C to stop.  To run:
# On TurtleBot:
# roslaunch turtlebot_bringup minimal.launch
# On work station:
# python goforward.py

import sys
import math
import rospy
import tf
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped

class Turtlebot():
    def __init__(self):
       
        
	# Create a publisher which can "talk" to TurtleBot and tell it to move
        # Tip: You may need to change cmd_vel_mux/input/navi to /cmd_vel if you're not using TurtleBot2
        self.cmd_vel = rospy.Publisher('cmd_vel/', Twist, queue_size=10)
        
        # Exercise 1: declare a transform listener, and get the parameters from the ROS parameter server
        
        
        # TODO: Retrieve parameters from parameter server
        self.base_frame_id = "base_footprint"
        self.global_frame_id = "odom"
        self.v_ref = 0.5
        
        # End of Exercise 1
        
        # Exercise 2: the goal should be received from the proper topic. Add a subscriber and implement a callback method
        self.goal_received = False
        self.goal = PoseStamped()
        self.goal.pose.orientation.w = 1.0
        self.goal.header.frame_id = "odom"
        self.goal.pose.position.x = 2.0
        self.goal.pose.position.y = 2.0
        self.goal.pose.position.z = 0.0
        # End of exercise 2
        
    def control_loop(self):
        rospy.loginfo("Turtlebot Controller: Control Loop")
        
        linear = 0.0
        angular = 0.0

        # TODO: Exercise 1a: Transform the goal to the local frame and implement the control loop
        base_goal = PoseStamped()
            
        # TODO: Exercise 2. Put the control law here (from the value of base_goal get the linear and angular velocity commands)
        
        # TODO: Exercise 2a. Use a proportional control to calculate the angular velocity command from the angular error
            
        # TODO: Exercise 2b. calculate the linear velocity command with trapezoidal profile (first you could try constant velocity)
        
        # TODO: Exercise 2c. Stop when the robot is close enough to the goal
        
        # End of Exercise 2a - 2b
    
        self.publish(linear,angular)

    def publish(self, lin_vel, ang_vel):
	    # Twist is a datatype for velocity
        move_cmd = Twist()
	    # Copy the forward velocity
        move_cmd.linear.x = lin_vel
	    # Copy the angular velocity
        # move_cmd.angular.z = ang_vel
        self.cmd_vel.publish(move_cmd)
        
    def shutdown(self):
        # stop turtlebot
        rospy.loginfo("Stop TurtleBot")
	    # a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
        self.cmd_vel.publish(Twist())
	    # sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
        rospy.sleep(1)
 
if __name__ == '__main__':
    try:
	 # initiliaze
        rospy.init_node('turtlebot_controller', anonymous=False)

	    # tell user how to stop TurtleBot
        rospy.loginfo("Initializing Turtlebot Controller. Please press CTRL + C to stop TurtleBot ")

        robot=Turtlebot()
	    # What function to call when you ctrl + c    
        rospy.on_shutdown(robot.shutdown)

	    #TurtleBot will stop if we don't keep telling it to move.  How often should we tell it to move? 10 HZ
        r = rospy.Rate(10)

	    # as long as you haven't ctrl + c keeping doing...
        while not rospy.is_shutdown():
            rospy.loginfo("Loop")
            # publish the velocity
            robot.control_loop()
            # wait for 0.1 seconds (10 HZ) and publish again
            r.sleep()

    except:
        rospy.loginfo("robotcontrol node terminated.")
