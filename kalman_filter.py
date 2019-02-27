"""
Roboxi - Kalman Filter

Author: Raphael Kusumoto Barbosa de Almeida
E-Mail : raphael_kba@hotmail.com
"""
import numpy as np
import copy



class Kalman_filter():
    def __init__(self, states_size):
        self.Q = np.ones((states_size,states_size))
        self.Q[0,0] = 2.1
        self.Q[1,1] = 2.1        
#        self.Q[2,2] = np.deg2rad(1.0)
#        self.Q *= self.Q
        self.R = np.diag([1.0, 1.0])  
        self.P = np.eye(states_size)
        self.jacobi_obs = np.zeros((2,states_size))
        self.jacobi_obs[0,0] = 1.0
        self.jacobi_obs[1,1] = 1.0
        self.states_size = states_size
        
    def predict(self, filter_robot):
        add_noise = True
        filter_robot.run(add_noise)
        A, B = filter_robot.jacobi(0.0)
        p_predicted = A@self.P@A.T + self.Q
        return p_predicted

    def update(self, filter_robot, p_predicted, ground_truth):
        z_prediction = self.jacobi_obs@filter_robot.states
        z = self.sim_gps(ground_truth)        
        y = z - z_prediction
        S = self.jacobi_obs@p_predicted@self.jacobi_obs.T + self.R
        K = p_predicted@self.jacobi_obs.T@np.linalg.inv(S)
        filter_robot.states = filter_robot.states + K@y
        self.P = (np.eye(len(filter_robot.states)) - K@self.jacobi_obs)@p_predicted
        
        
    
    def run_filter(self, robot, ground_truth):
        filter_robot = copy.deepcopy(robot)     
        p_predicted = self.predict(filter_robot)
        self.update(filter_robot, p_predicted, ground_truth)
        return filter_robot

    
    def sim_gps(self, states):
        cov_gps= np.diag([0.5, 0.5])**2
        gps_states = np.zeros((2,1))
        gps_states[0] = states[0,0] + np.random.rand()*cov_gps[0,0]
        gps_states[1] = states[1,0] + np.random.rand()*cov_gps[1,1]
        return gps_states