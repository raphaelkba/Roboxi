"""
Robotic Controls Class

Author: Raphael Kusumoto Barbosa de Almeida
E-Mail : raphael_kba@hotmail.com
"""

import numpy as np
import math
import scipy.linalg as la

from utils import utils

class controllers():
    
    def __init__(self, gains):
        self.gains = gains
        self.error_old = 0


    def pose_control(self, robot):
        
        robot.goal = self.look_ahead(robot)
        
        # distance to goal
        distance = utils.euclidean_distance(robot.states, robot.goal)
               
        # PID controller for velocity
        velocity_control = self.gains[0]*distance
        # angle control
        angle_to_point = (utils.angle_between_points(robot.states, robot.goal) - robot.states[2][0] + math.pi)%(2*math.pi) - math.pi
        beta = (robot.goal[2] - robot.states[2][0] - angle_to_point + math.pi) % (2 * math.pi) - math.pi

        # derivative term for angle control         
        error_diff = angle_to_point - self.error_old
        self.error_old = angle_to_point

        # maneuver
        if angle_to_point > math.pi / 2 or angle_to_point < -math.pi / 2:
            velocity_control = -velocity_control

        return np.array([velocity_control, self.gains[2]*angle_to_point + self.gains[3]*beta + self.gains[1]*error_diff])        
 
 
    def lqr_steer_control(self, robot):
        self.Q = np.eye(4)
        self.R = np.eye(2)
        robot.goal = self.look_ahead(robot)
        

        ######## Linearization ########
        vel = utils.euclidean_distance(robot.states, robot.goal)
        A, B = robot.jacobi()

        ######## DARE ########
        X = self.solve_DARE(A, B)
        
        ######## LQR solution ########
        lqr_k = self.solve_lqr(X, A, B)
            
        error = robot.error() 
        
        ustar = -lqr_k @ error
        # calc steering input
        delta = utils.truncate_angle(ustar[1])
                
        return np.array([vel, delta])  
        
    def lqr_vel_steer_control(self, robot):
        robot.lookahead_idx = 25
        self.Q = np.eye(5)
        self.R = np.eye(2)
        robot.goal = self.look_ahead(robot)
        

        ######## Linearization ########
        A, B = robot.jacobi()

        ######## DARE ########
        X = self.solve_DARE(A, B)
        
        ######## LQR solution ########
        lqr_k = self.solve_lqr(X, A, B)
            
        error = robot.error() 
        
        ustar = -lqr_k @ error
        # calc steering input
        delta = utils.truncate_angle(ustar[1])
                
        return np.array([ustar[0], delta])  
        
    def solve_DARE(self, A, B):
        X = self.Q
        maxiter = 300
        eps = 0.001
    
        for i in range(maxiter):
            
            X_ = np.dot(np.dot(A.T, X), A) -\
            np.dot(np.dot(np.dot(np.dot(np.dot(np.dot(A.T, X), B), \
            np.linalg.inv(self.R + np.dot(np.dot(B.T, X), B))), B.T), X),A) \
            + self.Q  #@

            if (abs(X_ - X)).max() < eps:
                break
            X = X_
        return X
    
    def solve_lqr(self, X, A, B):
        return np.dot(np.linalg.inv(np.dot(np.dot(B.T, X),B) + self.R), np.dot(np.dot(B.T,X),A))
    
    def look_ahead(self, robot):    
        """
        search for the closest point on the reference path to the robot and return 
        a look ahed point (a point a little further than the closest one)
        """
        min_dist = float('inf')
        dist = 0.0
        idx = 0
        goal = [0.0, 0.0, 0.0]
        for i in range(len(robot.path[0])-robot.lookahead_idx): # find closest path point 
            dist = utils.euclidean_distance((robot.path[0][i],robot.path[1][i]), robot.states)
            if dist < min_dist:
                min_dist = dist
                idx = i
                goal[0] = robot.path[0][i+robot.lookahead_idx]
                goal[1] = robot.path[1][i+robot.lookahead_idx]
                goal[2] = math.atan2(robot.path[1][i+2]-robot.path[1][i], robot.path[0][i+2]- robot.path[0][i])
                if len(robot.path[0]) < robot.lookahead_idx+1: # if close to final goal, returns final goal 
                    goal[0] = robot.path[0][-1]
                    goal[1] = robot.path[1][-1]
                    goal[2] = math.atan2(robot.path[1][i+1]-robot.path[1][i], robot.path[0][i+1]- robot.path[0][i])
        for i in range(idx): # delete passed path points
            robot.path = np.delete(robot.path,[i],1)
        return goal
        
        