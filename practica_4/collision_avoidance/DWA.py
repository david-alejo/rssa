import rclpy
from rclpy import Node

#!/usr/bin/python3

# This node implements the Dynamic Window Approach to
# reactively avoid collisions with the obstacles from a LiDAR

import math
from math import cos, sin

import py
import rclpy
import tf2_ros
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan

class DWA(Node):
    def __init__(self): 
        #Ejercicio 1
        super().__init__('DWA')
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

        self.declare_parameter('v_max', 0.5)
        self.v_max = self.get_parameter('v_max').get_parameter_value()

        self.declare_parameter('delta_t', 0.1)
        self.delta_t = self.get_parameter('delta_t').get_parameter_value()

        self.declare_parameter('T', 2.0)
        self.T = self.get_parameter('T').get_parameter_value()

        

        # TODO: get the rest of parameters

        # example of obstacles (in reality they should be detected by the LiDAR and the callback)
        # should fill this attribute
        self.obstacles = [(2, 3), (4, 5), (6, 4)]

        print("DWA node initialized. V_Max: ", self.v_max)
        
    def control_loop(self):
        if self.v_pref is None:
            self.get_logger().debug("No velocity command received yet.")
            return
        
        ### TODO: call the DWA method for obtaining the optimal velocity (calling steps 2, 3 and 4)
        if self.emergency_stop:
            print("Stopping the robot")
        print("Publishing velocity command: linear.x = %.2f, angular.z = %.2f".format(self.v_pref.twist.linear.x, self.v_pref.twist.angular.z))
        self.cmd_vel_pub.publish(self.v_pref)
        ## End of task
        
        print("DWA control loop")


    def vPrefCallback(self, msg):
        # Hint: If no path is received it is set to None in the constructor (path received) 
        # And also reset the current wp counter (you can set it to the closest point to the robot)
        self.v_pref = msg

    def calculate_window(self, v_0):
        # Devuelve el conjunto de velocidades a probar [0.2, 0.1] [0.25,0.1] [0.3,0.1] ...

        #Ejercicio 2
        return [v_0]
    
    def simulate_trajectory(self, v, omega):
        # Devuelve la trayectoria simulada con paso delta_t, y horizonte T
        # Ejercicio 3
        
        #Al trabajar con coordenadas locales: x0, y0 y theta0 van a ser 0
        x = 0.0
        y = 0.0
        theta = 0.0
        t = 0.0
        traj = []

        delta_t = self.delta_t

        # Integramos la trajectoria
        while t < self.T:
            t += self.delta_t

            x += v*cos(theta) * delta_t
            y += v*sin(theta) * delta_t
            theta += omega * self.delta_t

            traj.append({x,y,theta})

        #Sugerencia: representar las trayectorias usando Markers.
        # ver: https://docs.ros.org/en/jazzy/Tutorials/Intermediate/RViz/Marker-Display-types/Marker-Display-types.html

    def evaluate_traj(self, obstacles, trajectory, goal):
        # Ejercicio 4
        # Peso del objetivo:
        x_f = trajectory[-1][0]
        y_f = trajectory[-1][1]
        
        # TODO: J_goal es la distancia a goal (de x_f e y_f a goalx y goaly)

        # Bucle para calcular distancia minima

        return -1.0

    def DWA(self):
        #Llama a 2, 3 y 4 para calcular la v y omega optimas
        print("TODO: implement DWA")


    def scanCallback(self, msg):
        self.get_logger().debug("Scan received. Number of points: %d", len(msg.ranges))
        self.emergency_stop = False
        
        print("Here")
        
        angle = msg.angle_min
        for distance in msg.ranges:
            #### Move to range -PI, PI
            if angle > math.pi:
                angle -= 2*math.pi
            
            # TODO: update the obstacles attribute of the class with each obstacle
             
            ## End of task
            angle += msg.angle_increment
            # end of Exercise 1

def main(args=None):
        rclpy.init(args=args)
        #try:
	    # initiliaze
	    # tell user how to stop TurtleBot
        print("Emergency stop node. It will stop the robot in case of an imminent collision.")

        node = DWA()
	    # What function to call when you ctrl + c    

	    
	    # as long as you haven't ctrl + c keeping doing...
        rclpy.spin(node)

 
if __name__ == '__main__':
    main()
    