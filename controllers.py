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

        self.e, self.th_e = 0.0, 0.0
        self.pe, self.pth_e = 0.0, 0.0

    def simple_control(self, gains, states, reference):
        
        # distance to goal
        error = utils.euclidean_distance(states, reference)
        # derivative term        
        
        # integral term
        self.error_sum += error
        
        # PID controller for velocity
        velocity_control = gains[0]*error
        # angle control
        angle_to_point = (utils.angle_between_points(states, reference) - states[2] + math.pi)%(2*math.pi) - math.pi
        error_diff = angle_to_point - self.error_old
        self.error_old = angle_to_point
        # integral term
        self.error_sum += angle_to_point
        beta = (reference[2] - states[2] - angle_to_point + math.pi) % (2 * math.pi) - math.pi
        # maneuver
        if angle_to_point > math.pi / 2 or angle_to_point < -math.pi / 2:
            velocity_control = -velocity_control

        
        return np.array([velocity_control, gains[3]*angle_to_point + gains[4]*beta + gains[1]*error_diff + gains[2]*self.error_sum])
        
    def LQR(self, states, reference, dt):
        
        ######## Linearization ########
        L = 0.1
        r = 0.05
        
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
            
        angle_to_point = (utils.angle_between_points(states, reference) - states[2][0] + math.pi)%(2*math.pi) - math.pi
        error = [[0],[0],[0]]
        error[0] = states[0][0]
        error[1] = states[1][0]
        error[2] = states[2][0] - angle_to_point
        
        ustar = -lqr_k @ error
        
        # calc steering input
        delta = utils.truncate_angle(ustar[1])
                

        return np.array([ai, delta])
        
        
    def LQR_Bicycle(self, states, reference, dt):
        self.Q = np.eye(5)
        self.Q[0, 0] = 1.0
        self.Q[1, 0] = 1.0
#        self.Q[2, 0] = 50.0
#        self.Q[3, 0] = 0.1
        self.Q[4, 0] = 0.1
        self.R = np.eye(2)*0.01
        ######## Linearization ########
        L = 4.5
        r = 0.05
        
        theta = states[2][0]
        v = states [3][0]        
        phi = states[4][0]
        
        A = np.zeros((5, 5))
        A[0, 0] = 1.0
        A[0, 1] = 0.0
        A[0, 2] = -dt*v*np.sin(theta)
        A[0, 3] = dt*np.cos(theta)
        A[0, 4] = 0
        A[1, 0] = 0.0
        A[1, 1] = 1.0
        A[1, 2] = dt*v*np.cos(theta)
        A[1, 3] = dt*np.sin(theta)
        A[1, 4] = 0.0
        A[2, 0] = 0.0
        A[2, 1] = 0.0
        A[2, 2] = 1.0
        A[2, 3] = dt*np.tan(phi)/L
        A[2, 4] = dt*v/(np.cos(phi)*np.cos(phi)*L)
        A[3, 0] = 0.0        
        A[3, 1] = 0.0    
        A[3, 2] = 0.0
        A[3, 3] = 1.0        
        A[3, 4] = 0.0
        A[4, 0] = 0.0       
        A[4, 1] = 0.0        
        A[4, 2] = 0.0        
        A[4, 3] = 0.0        
        A[4, 4] = 1.0        
        
        B = np.zeros((5, 2))
        B[3, 0] = 1.0
        B[4, 1] = 1.0

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
        
        angle_to_point = (utils.angle_between_points(states, reference) - states[2][0] + math.pi)%(2*math.pi) - math.pi
        error = [[0],[0],[0],[0],[0]]
        error[0] = states[0][0] - reference[0]
        error[1] = states[1][0] - reference[1]
        error[2] = states[2][0] - angle_to_point
        error[3] = states[3][0]
        error[4] = states[4][0] - angle_to_point
        
        ustar = -lqr_k @ error
        
        # calc steering input
        delta = utils.truncate_angle(ustar[1])
        ai = ustar[0]

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
        
        
        
        