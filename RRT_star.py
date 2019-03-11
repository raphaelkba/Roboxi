"""
Robotic Simulation 
RRT* path planning
Source: https://arxiv.org/pdf/1105.1186.pdf
Author: Raphael Kusumoto Barbosa de Almeida
E-Mail : raphael_kba@hotmail.com
"""
import copy
import math
import random
import numpy as np
import matplotlib.pyplot as plt
from utils import utils
from planner import Planner
import matplotlib.patches as mpatches

infinity = float("inf")

class RRT_star(Planner):
    
    def __init__(self, start, goal, obstacles, 
                 map_limits, expand_distance, max_itr):
                     
        self.start = start
        self.goal = goal
        self.obstacles = obstacles
        self.expand_distance = expand_distance
        self.max_itr = max_itr
        self.nodes = []
        self.nodes.append([self.start[0,0], self.start[1,0], 0.0, None]) # x, y, cost, parent
        self.itr = 0
        self.dim = 2
        self.gamma = 30.0
        super().__init__(self.start, self.goal, map_limits)

        
    def plan(self):
        print("Start RRT*")
        itr = 0
        while itr < self.max_itr:
            print(itr)
            # sample free
            if random.randint(0, 100) > 20:
                x_rand = [random.uniform(self.min_lim_x, self.max_lim_x), random.uniform(
                    self.min_lim_y, self.max_lim_y)]
            else:
                x_rand = [self.goal[0], self.goal[1]]
            # nearest function
            closest_node, closest_idx = utils.find_closest_point(self.nodes, x_rand)    
            
            new_node = self.steer(x_rand, closest_node, closest_idx)
        
            if not self.obstacle_free(new_node):
                continue
            
            near_nodes_idx = self.near(new_node)
            near_nodes_costs = self.get_costs(new_node, near_nodes_idx)
            new_node = self.find_parent(near_nodes_costs, near_nodes_idx, new_node)
            self.nodes.append(new_node) 
            self.rewire_tree(near_nodes_idx, new_node)
#            if self.check_goal(new_node):
#                print("RRT* found a goal")
#                break
            itr += 1
        if self.planner_animation:
            self.animation()


        path = self.find_path()
        plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
        plt.show()
        path_x = np.array(path[:,0])
        path_y = np.array(path[:,1])
        path = [path_x[::-1], path_y[::-1]]
        return path
    
    def get_index(self):
        # find closest node to the goal with best cost 
        distance2goal = [utils.euclidean_distance(node, self.goal) for node in self.nodes]
        indexes = []
        
        for distance in distance2goal:
            if distance <= 2*self.expand_distance:
                indexes.append(distance2goal.index(distance))
                
        if indexes:
            mincost = min([self.nodes[i][2] for i in indexes])
            for idx in indexes:
                if self.nodes[idx][2] == mincost:
                    return idx

        return None

    def find_path(self):
        path = [[self.goal[0], self.goal[1]]]
        idx = self.get_index()
        while self.nodes[idx][3] is not None:
            path.append([self.nodes[idx][0], self.nodes[idx][1]])
            idx = self.nodes[idx][3]
        path.append([self.start[0,0], self.start[1,0]])
        return np.array(path)
    
                    
    def rewire_tree(self, near_nodes_idx, new_node):
        cardV = len(self.nodes)

        for idx in near_nodes_idx:
            node = self.nodes[idx]
            distance = utils.euclidean_distance(new_node, node)
            cost = new_node[2] + distance
            
            if node[2] > cost:
                if self.collision_free(new_node, node):
                    node[2] = cost
                    node[3] = cardV - 1
        
    
    # check if along the line between the two points are collisions
    def collision_free(self, node, parent):
#        node_ = [node[0], node[1]]
        parent_ = copy.deepcopy(parent)
        distance = utils.euclidean_distance(parent_, node)
        orientation = math.atan2(node[1] - parent_[1], node[0] - parent_[0])
        line = np.arange(0, distance, 0.1)
        for i in line:
            parent_[0] += self.expand_distance*math.cos(orientation)
            parent_[1] += self.expand_distance*math.sin(orientation)
            if not self.obstacle_free(parent_):
                return False
        return True
    

    def find_parent(self, costs, near_nodes_idx, new_node):
        if near_nodes_idx:
            minimum_cost = np.min(costs)
            index_min = np.argmin(costs)
            if minimum_cost != infinity:
                new_node[2] = minimum_cost
                new_node[3] = near_nodes_idx[index_min]
        return new_node
    
    def get_costs(self, new_node, near_nodes_idx):
        cost_list = []
        if near_nodes_idx:
            for idx in near_nodes_idx:
                dx = new_node[0] - self.nodes[idx][0]
                dy = new_node[1] - self.nodes[idx][1]
                d = math.sqrt(dx ** 2 + dy ** 2)
                theta = math.atan2(dy, dx)
                distance = utils.euclidean_distance(new_node, self.nodes[idx])
                orientation = math.atan2(new_node[1] - self.nodes[idx][1], new_node[0] - self.nodes[idx][0])
                if distance!= d or orientation!=theta:
                    pass
                if self.collision_free(new_node, self.nodes[idx]):
                    cost_list.append(self.nodes[idx][2] + utils.euclidean_distance(new_node, self.nodes[idx])) 
                else: 
                    cost_list.append(infinity)
        return cost_list
    
    
    def near(self, new_node):
        cardV = len(self.nodes)
        r = self.gamma*((np.log(cardV) / cardV)**(1/self.dim))
        distances = [(node[0] - new_node[0]) ** 2 +
                 (node[1] - new_node[1]) ** 2 for node in self.nodes]
        near_nodes_idx = []
        for i in range(len(distances)):
            if distances[i] <= r*r:
                near_nodes_idx.append(i)
        return near_nodes_idx
    
    def steer(self, xrand, closest_node, closest_idx):
        orientation = math.atan2(xrand[1] - closest_node[1], xrand[0] - closest_node[0]) 
        distance = utils.euclidean_distance(xrand, closest_node)
        new_node = [xrand[0], xrand[1], infinity, None]
        if distance > self.expand_distance: 
            new_node[0] = closest_node[0] + self.expand_distance*math.cos(orientation)
            new_node[1] = closest_node[1] + self.expand_distance*math.sin(orientation)
        return new_node                     
                           
    def obstacle_free(self, node):
        for obstacle in self.obstacles:
            if ((obstacle[0] - node[0])*(obstacle[0] - node[0]) + (obstacle[1] - node[1])*(obstacle[1] - node[1])) <= obstacle[2]**2:
                return False
        return True
    
    def check_goal(self, node):
        if ((self.goal[0] - node[0])** 2 + (self.goal[1] - node[1])** 2) < self.expand_distance:
            return True
    

        
    def animation(self):
        for obstacle in self.obstacles:
            obs = mpatches.FancyBboxPatch((obstacle[0]-obstacle[2]/2, obstacle[1]-obstacle[2]/2), obstacle[2], obstacle[2], boxstyle=mpatches.BoxStyle("Round", pad=0.01),color='red')  
            self.ax.add_patch(obs)   
            
        self.ax.plot(self.goal[0], self.goal[1], "xr")            
        self.ax.plot(self.start[0,0], self.start[1,0], "ob")
        
        for node in self.nodes:
            if node[3] != None:
                plt.plot([node[0], self.nodes[node[3]][0]], [node[1], self.nodes[node[3]][1]], "-k")
        
#        self.ax.axis('equal')

        self.ax.set_xlim(-20, 20)
        self.ax.set_ylim(-20, 20)
       
        plt.xlabel('x [m]')
        plt.ylabel('y [m]')
        plt.title('RRT Planner')
#        if self.itr%5 == 0:
#        plt.savefig("images/"+ str(self.itr) +".png")
        self.itr += 1
        plt.pause(0.001)


