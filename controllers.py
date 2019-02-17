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
    
    def __init__(self):


        self.error_old = 0.0
        self.error_sum = 0.0
        states_size = 3
        controls_size = 2
        self.Q = np.eye(states_size)
        self.R = np.eye(controls_size)
        self.e, self.th_e = 0.0, 0.0
        self.pe, self.pth_e = 0.0, 0.0

    def simple_control(self, gains, states, reference):
        
        # distance to goal
        error = utils.euclidean_distance(states, reference)
        # derivative term        
        error_diff = error - self.error_old
        # integral term
        self.error_sum += error
        self.error_old = error
        # PID controller for velocity
        PID = gains[0]*error + gains[1]*error_diff + gains[2]*self.error_sum
        # angle control
        angle_to_point = (utils.angle_between_points(states, reference) - states[2] + math.pi)%(2*math.pi) - math.pi

        beta = (reference[2] - states[2] - angle_to_point + math.pi) % (2 * math.pi) - math.pi
        # maneuver
        if angle_to_point > math.pi / 2 or angle_to_point < -math.pi / 2:
            PID = -PID

        
        return np.array([PID, gains[3]*angle_to_point + gains[4]*beta])
        
    def LQR(self, states, reference, dt):
        
        ######## Linearization ########
        L = 0.1
        r = 0.05
        v = states[0][0] + states[1][0]
        theta = states[2][0]
        
        ai = utils.euclidean_distance(states, reference)
        v = ai
        A = np.zeros((3, 3))
        A[0, 0] = 1.0
        A[0, 1] = 0.0
        A[0, 2] = -dt*r*v*np.sin(theta)
        A[1, 0] = 0.0
        A[1, 1] = 1.0
        A[1, 2] = dt*r*v*np.cos(theta)
        A[2, 0] = 0.0
        A[2, 1] = 0.0
        A[2, 2] = 1.0

        
        B = np.zeros((3, 2))
        B[0, 0] = dt*r*np.cos(theta)
        B[1, 0] = dt*r*np.sin(theta)
        B[2, 1] = (dt*r)/L

        
        ######## DARE ########
        X = self.Q
        maxiter = 150
        eps = 0.01
    
        for i in range(maxiter):
            
            X_ = np.dot(np.dot(A.T, X), A) -\
            np.dot(np.dot(np.dot(np.dot(np.dot(np.dot(A.T, X), B), \
            np.linalg.inv(self.R + np.dot(np.dot(B.T, X), B))), B.T), X),A) \
            + self.Q  #@

            if (abs(X_ - X)).max() < eps:
                break
            X = X_
        
        ######## LQR solution ########
        lqr_k = np.dot(np.linalg.inv(np.dot(np.dot(B.T, X),B) + self.R), np.dot(np.dot(B.T,X),A))
        K = la.inv(B.T @ X @ B + self.R) @ (B.T @ X @ A)
            
        angle_to_point = (utils.angle_between_points(states, reference) - states[2][0] + math.pi)%(2*math.pi) - math.pi
        error = [[0],[0],[0]]
        error[0] = states[0][0]
        error[1] = states[1][0]
        error[2] = states[2][0] - angle_to_point
        
        ustar = -lqr_k @ error
        
        # calc steering input
        delta = utils.truncate_angle(ustar[1])
                

        return np.array([ai, delta])
        
    def look_ahead(self, path, goal_final, states, lookahead_idx):
        min_dist = float('inf')
        dist = 0
        goal = [0,0,0]
        for i in range(len(path[0])-lookahead_idx):
            dist = utils.euclidean_distance((path[0][i],path[1][i]), states)
            if dist < min_dist:
                min_dist = dist
                idx = i
                goal[0] = path[0][i+lookahead_idx]
                goal[1] = path[1][i+lookahead_idx]
                if len(path[0]) > lookahead_idx+1:
                    goal[2] = math.atan2(path[1][i+int(2)]-path[1][i], path[0][i+int(2)]- path[0][i])
                else:
                    goal[0] = goal_final[0]
                    goal[1] = goal_final[1]
                    goal[2] = math.atan2(path[1][i+int(2)]-path[1][i], path[0][i+int(2)]- path[0][i])
        for i in range(idx):
            path = np.delete(path,[i],1)
        return path, goal
        
        
        
        