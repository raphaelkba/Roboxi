import numpy as np
import math

class utils():
    
    def __init__(self):
        pass
    
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