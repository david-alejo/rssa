"""
implement here your path planning method
"""

import math
from nav_msgs.msg import OccupancyGrid


class Planner:
    ####### Class that represents a node ######
    class Node:
        def __init__(self, cx, cy, cost, parent):
            self.x_cell = cx  # x index in the obstacle grid
            self.y_cell = cy  # y index in the obstacle grid
            
            

            # TODO: add the node costs that you may need
            self.cost = cost #?
            # self.g?
            # self.f?

            self.parent = parent  # index of the previous Node
            
    ###### Methods of Planner class ####
    def __init__(self, costmap):
        """ 
        Initialize a map from a ROS costmap
        
        costmap: ROS costmap
        """
        # Copy the map metadata
        self.resolution = costmap.info.resolution
        self.min_x = costmap.info.origin.position.x
        self.min_y = costmap.info.origin.position.y
        self.y_width = costmap.info.height
        self.x_width = costmap.info.width
        self.max_x = self.min_x + self.x_width * self.resolution
        self.max_y = self.min_y + self.y_width * self.resolution
        print("min corner x: %.2f m, y: %.2f m", self.min_x, self.min_y)
        print("max corner x: %.2f m, y: %.2f m", self.max_x, self.max_y)
        print("Resolution: %.3f m/cell", self.resolution)
        print("Width: %i cells, height: %i cells", self.x_width, self.y_width)
        
        self.motion = self.get_motion_model()
        
        # Copy the actual map data from the map
        x = 0
        y = 0
         # obstacle map generation
        self.obstacle_map = [[False for _ in range(self.y_width)]
                             for _ in range(self.x_width)]
        obstacles = 0
        for value in costmap.data:
            if value > 80:  # This value could change depending on the map
                obstacles += 1
                self.obstacle_map[x][y] = True
            # Update the iterators
            x += 1
            if x == self.x_width:
                x = 0
                y += 1
        print("Loaded %d obstacles"%(obstacles))

        # NOTE: alternatively, instead of computing a binary 
        # obstacle map, you can try to store directly the costmap 
        # values and use them as costs.
        

    def plan(self, sx, sy, gx, gy):
        """
        Fill with your search method

        input:
            sx: start x position [m]
            sy: start y position [m]
            gx: goal x position [m]
            gx: goal x position [m]

        output:
            rx: x position list of the final path
            ry: y position list of the final path
        """
        
        # first check if we are already very close
        d = math.sqrt((gx-sx)*(gx-sx) + (gy-sy)*(gy-sy))
        if d <= self.resolution*2.0:
            return [], []

        # create the start node and the goal node
        start_cell_x, start_cell_y = self.real2cell(sx, sy)  
        start_node = self.Node(start_cell_x, start_cell_y, 0.0, -1)
        goal_cell_x, goal_cell_y = self.real2cell(gx, gy)
        goal_node = self.Node(goal_cell_x, goal_cell_y, 0.0, -1)

        # check if the positions are valid (no obstacle)
        if (not self.is_valid(start_node)):
            print("Error: start position not valid!!")
            return [], []
        
        if (not self.is_valid(goal_node)):
            print("Error: goal position not valid!!")
            return [], []
        
        # TODO: fill the rest of the function
        
        print(start_node, goal_node)

        open_set, closed_set = dict(), dict()
        open_set[self.node2index(start_node)] = start_node

        # These lists will store the path
        pathx = [] 
        pathy = [] 
        
        while len(pathx) == 0:
            # Get the current index:
            c_id = min(open_set, key=lambda o: open_set[o].cost)
            current = open_set[c_id]
            
            # Check if we have reached the goal
            if current.x_cell == goal_node.x_cell and current.y_cell == goal_node.y_cell:
                print ("Goal found!!!")
                goal_node.parent = current.parent
                goal_node.cost = current.cost
                pathx, pathy = self.calc_final_path(goal_node, closed_set)
                
            else:
                # Delete from the openset and add to closed set
                del open_set[c_id]
                closed_set[c_id] = current
                # For all possible moves
                for m_x, m_y, cost in self.motion:
                    new = self.Node(current.x_cell + m_x,
                                     current.y_cell + m_y,
                                     current.cost + cost,
                                     c_id)
                    n_id = self.node2index(new)
                    
                    if n_id in open_set:
                        if open_set[n_id].cost >= new.cost:
                        # This path is the best until now. record it!
                            open_set[n_id] = new
                    elif n_id not in closed_set and self.is_valid(new):
                        open_set[n_id] = new        
        
        # return the path
        return pathx, pathy


    # Transform map coordinates in meters
    # to cell indexes in the grid
    def real2cell(self, rx, ry):
        cellx = round((rx - self.min_x) / self.resolution)
        celly = round((ry - self.min_y) / self.resolution)
        return cellx, celly

    # Tranform cell indexes of the grid
    # to map coordinates in meters
    def cell2real(self, cx, cy):
        rx = cx * self.resolution + self.min_x
        ry = cy * self.resolution + self.min_y
        return rx, ry

    # Associate each pixel with a unique id (index)
    # to make it manageable by sets
    def real2index(self, rx, ry):
        ret_val = -1
        if rx >= self.min_x and ry >= self.min_y and \
           rx < self.max_x and ry < self.max_y:
            
            cx, cy = self.real2cell(rx, ry)
            ret_val = self.cell2index(cx, cy)
            
        return ret_val
    
    def cell2index(self, cx, cy):
        return cx * self.y_width + cy
    
    def node2index(self, node:Node):
        return self.cell2index(node.x_cell, node.y_cell)
    
    ### Retrieve the path from the goal to the init (reverse!)
    def calc_final_path(self, goal_node:Node, closed_set):
        # generate final course
        
        
        rx, ry = self.cell2real(goal_node.x_cell, goal_node.y_cell)
        pathx = [rx]
        pathy = [ry]
        parent = goal_node.parent
        while parent != -1:
            n = closed_set[parent]
            rx, ry = self.cell2real(n.x_cell, n.y_cell)
            pathx.append(rx)
            pathy.append(ry)
            parent = n.parent
        pathx.reverse() # Do not forget to reverse it
        pathy.reverse()
        return pathx, pathy

    def is_valid(self, node):

        # check that the node is inside the grid limits
        rx, ry = self.cell2real(node.x_cell, node.y_cell)
        if rx < self.min_x:
            return False
        if ry < self.min_y:
            return False
        if rx >= self.max_x:
            return False
        if ry >= self.max_y:
            return False
        
        # check if the cell is an obstacle
        return not self.obstacle_map[int(node.x_cell)][int(node.y_cell)]

    @staticmethod
    def get_motion_model():
        # dx, dy, cost
        # TODO 2: You could include further motion so that we can derive to Theta* algorithm
        # TODO 2: Note that in those cases, we should check for line of sight (when the two nodes are far apart)
        motion = [[1, 0, 1],
                [0, 1, 1],
                [-1, 0, 1],
                [0, -1, 1],
                [-1, -1, math.sqrt(2)],
                [-1, 1, math.sqrt(2)],
                [1, -1, math.sqrt(2)],
                [1, 1, math.sqrt(2)]]

        return motion




