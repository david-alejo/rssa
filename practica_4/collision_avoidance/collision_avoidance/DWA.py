import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan
import math
import numpy as np

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

        self.declare_parameter('min_d', 0.2)
        self.min_d = self.get_parameter('min_d').value
        self.min_d_sq = self.min_d**2

        self.declare_parameter('delta_t', 0.1)
        self.delta_t = self.get_parameter('delta_t').value

        self.declare_parameter('T', 4.0)
        self.T = self.get_parameter('T').value

        self.declare_parameter('base_frame','base_footprint')
        self.base_frame_id = self.get_parameter('base_frame').get_parameter_value()

        # TODO: get the rest of parameters

        # example of obstacles (in reality they should be detected by the LiDAR and the callback)
        # should fill this attribute
        self.obstacles = [(2, 3), (4, 5), (6, 4)]

        print("DWA node initialized. Min_d:", self.min_d,"\tV_Max: ", self.v_max)
        
    def control_loop(self):
        if self.v_pref is None:
            self.get_logger().debug("No velocity command received yet.")
            return
        
        ### TODO: call the DWA method for obtaining the optimal velocity (calling steps 2, 3 and 4)
        

        vx = self.v_pref.twist.linear.x
        wz = self.v_pref.twist.angular.z

        vx, wz = self.DWA(vx, wz)
        
        v_cmd = self.v_pref
        v_cmd.twist.linear.x = vx
        v_cmd.twist.angular.z = wz

        self.get_logger().info("Publishing velocity command: linear.x = " + str(vx) + "angular.z = " + str(wz))

        self.cmd_vel_pub.publish(v_cmd)
        
        ## End of task
        

    def vPrefCallback(self, msg):
        # Hint: If no path is received it is set to None in the constructor (path received) 
        # And also reset the current wp counter (you can set it to the closest point to the robot)
        self.v_pref = msg

    def calculate_window(self, v, w):
        # Devuelve el conjunto de velocidades a probar [0.2, 0.1] [0.25,0.1] [0.3,0.1] ...

        velocities = [(v,w)]
        
        #TODO: Complete this function to return a set of velocities to test, given the current velocity (v,w) 
        # and the max acceleration of the robot.


        #Ejercicio 2
        return velocities
    
    def evaluate_traj(self, v, omega, v_ref, omega_ref):
        # Simula la trayectoria simulada con paso delta_t, y horizonte T
        # Y devuelve una puntuación de la trayectoria simulada,
        # teniendo en cuenta la distancia a los obstáculos y
        # la cercanía a la velocidad objetivo (v_ref, omega_ref)
        # Ejercicio 3
        
        #Al trabajar con coordenadas locales: x0, y0 y theta0 van a ser 0
        x = 0.0
        y = 0.0
        theta = 0.0
        t = 0.0
        traj = []

        min_dist = -1.0

        delta_t = self.delta_t

        choca = False

        # Integramos la trajectoria
        while t < self.T:
            t += self.delta_t

            x += v*cos(theta) * delta_t
            y += v*sin(theta) * delta_t
            theta += omega * self.delta_t

            traj.append((x,y,theta))

            # TODO: Comprobar contra los obstáculos (Mucho más rápido)
            # Para levantar la bandera choca y calcular la distancia al obstáculo más cercano (min_dist)
            

        #Sugerencia: representar las trayectorias usando Markers.
        # ver: https://docs.ros.org/en/jazzy/Tutorials/Intermediate/RViz/Marker-Display-types/Marker-Display-types.html

        # TODO: Evaluacion: tres criterios: 
        # w mas cercano, v mas cerano
        # mayor mínima distancia a los obstáculos.
        #  Se pueden dos de ellos con dos pesos: alpha y beta  
        score = -10000.0

        return score

    def DWA(self, v, w):
        #Llama a 2, 3 y 4 para calcular la v y omega optimas
        # TODO: Implementar esta función para llamar a calculate_window y evaluate_traj, y devolver la mejor velocidad (v,w) para evitar colisiones y acercarse a la velocidad objetivo (v_ref, omega_ref)
        if (v > -0.001 and v < 0.001):
            return 0.0, w
        
        # Primero calculamos el conjunto de velocidades a probar    
        test_set = self.calculate_window(v, w)

        max_score = -1e9

        best = test_set[0]

        #TODO: iterar sobre el conjunto de velocidades a probar, evaluar cada una de las trayectorias simuladas y quedarnos con la mejor (la que tenga mayor puntuación)
        


        # TODO: ¿Qué pasa si no hay ninguna trayectoria segura? (puntuación muy baja) 
        # Deberíamos parar el robot, o ir a la velocidad más baja posible (0.05 m/s), o girar aleatoriamente
        # para intentar salir de la situación de bloqueo. 
        # Implementar alguna de estas estrategias para evitar que el robot se quede bloqueado sin moverse.
        if max_score < -1000.0:
            best = (0.0, 0.2) # Probablemente según donde esté el obstáculo, lo mejor sea girar al otro lado. Probar diferentes estrategias.

        return best

    def scanCallback(self, msg):
        self.emergency_stop = False
        
        self.obstacles = []

        angle = msg.angle_min
        i = 0
        for r in msg.ranges:
            #### Move to range -PI, PI
            if angle > math.pi:
                angle -= 2*math.pi
            
            # TODO: update the obstacles attribute of the class with each obstacle
            if i%2 == 0 and angle > -math.pi*0.5 and angle < math.pi *0.5 and r < 2.0:
                self.obstacles.append((r * math.cos(angle), r * math.sin(angle)))

            ## End of task
            angle += msg.angle_increment
            i += 1
        self.get_logger().info("Scan received. Number of obstacles:" + str( len(self.obstacles)))
        
        # end of Exercise 1

def main(args=None):
        rclpy.init(args=args)
        #try:
	    # initiliaze
	    # tell user how to stop TurtleBot
        print("DWA node. It implements a basic Dynamic Window Approach based collision avoidance algorithm.")

        node = DWA()
	    # What function to call when you ctrl + c    

	    
	    # as long as you haven't ctrl + c keeping doing...
        rclpy.spin(node)

 
if __name__ == '__main__':
    main()
    