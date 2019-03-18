"""
Roboxi - Utils

Author: Raphael Kusumoto Barbosa de Almeida
E-Mail : raphael_kba@hotmail.com
"""

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
import math

class utils():
    
    def euclidean_distance(vec1, vec2):
        #Computes the euclidean distance between two points x and y.
        x1, y1 = vec1[0:2]
        x2, y2 = vec2[0:2]
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    
    def angle_between_points(vec1, vec2):
        #Computes the angle between two points in radians
        x1, y1 = vec1[0:2]
        x2, y2 = vec2[0:2]
        angle = math.atan2(y2 - y1, x2 - x1)
        return angle
        
    # helper function to map all angles onto [-pi, pi]
    def truncate_angle(angle):
        while angle < 0.0:
            angle += math.pi * 2
        return ((angle + math.pi) % (math.pi * 2)) - math.pi
    
    def constrain_value(value, min_value, max_value):
        if value < min_value:
            value = min_value
        elif value > max_value:
            value = max_value
        return value

    def find_closest_point(nodes, curr_node): # change vars to vec and return only min index

        distances = [(node[0] - curr_node[0]) ** 2 + (node[1] - curr_node[1])
                 ** 2 for node in nodes]
        return nodes[distances.index(min(distances))], distances.index(min(distances))
        
    
    def collision_square_obstacle(pos, obstacles):
        for obstacle in obstacles:
            x_min = obstacle[0] - obstacle[2]/2
            x_max = obstacle[0] + obstacle[2]/2
            y_min = obstacle[1] - obstacle[2]/2
            y_max = obstacle[1] + obstacle[2]/2
            if x_min <= pos[0] and pos[0] <=  x_max and y_min <= pos[1] and pos[1] <= y_max:
                return True
        return False
    
    def collision_round_obstacle(pos, obstacles):
        for obstacle in obstacles:
             if ((obstacle[0] - pos[0])*(obstacle[0] - pos[0]) + (obstacle[1] - pos[1])*(obstacle[1] - pos[1])) <= obstacle[2]**2:
                return True
        return False

