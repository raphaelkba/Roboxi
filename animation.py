"""
Robotic Animation Class
Author: Raphael Kusumoto Barbosa de Almeida
E-Mail : raphael_kba@hotmail.com
"""
import numpy as np
import math
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from matplotlib import transforms
from scipy import ndimage
import matplotlib.transforms as mtransforms

class animation():
    
    def __init__(self):
        self.fig, self.ax = plt.subplots()

    
    
    def plot_vehicle(self, x, y, theta, x_traj, y_traj, dt, beta, df):  # pragma: no cover
        # Corners of triangular vehicle when pointing to the right (0 radians)
    # mudar  os nomes e td mais pra nao ser copia
        p1_i = np.array([0.5, 0, 1]).T
        p2_i = np.array([-0.5, 0.25, 1]).T
        p3_i = np.array([-0.5, -0.25, 1]).T
    
        T = transformation_matrix(x, y, theta)
        p1 = np.matmul(T, p1_i)
        p2 = np.matmul(T, p2_i)
        p3 = np.matmul(T, p3_i)
    
        T_b = transformation_matrix(x, y, beta)
        p1_b = np.matmul(T_b, p1_i)
        p2_b = np.matmul(T_b, p2_i)
        p3_b = np.matmul(T_b, p3_i)
    
        T_d = transformation_matrix(x, y, df)
        p1_d = np.matmul(T_d, p1_i)
        p2_d = np.matmul(T_d, p2_i)
        p3_d = np.matmul(T_d, p3_i)
    
        plt.plot([p1[0], p2[0]], [p1[1], p2[1]], 'k-')
        plt.plot([p2[0], p3[0]], [p2[1], p3[1]], 'k-')
        plt.plot([p3[0], p1[0]], [p3[1], p1[1]], 'k-')
    
        plt.plot([p1_b[0], p2_b[0]], [p1_b[1], p2_b[1]], 'r-')
        plt.plot([p2_b[0], p3_b[0]], [p2_b[1], p3_b[1]], 'r-')
        plt.plot([p3_b[0], p1_b[0]], [p3_b[1], p1_b[1]], 'r-')
    
        plt.plot([p1_d[0], p2_d[0]], [p1_d[1], p2_d[1]], 'g-')
        plt.plot([p2_d[0], p3_d[0]], [p2_d[1], p3_d[1]], 'g-')
        plt.plot([p3_d[0], p1_d[0]], [p3_d[1], p1_d[1]], 'g-')
    
        plt.plot(x_traj, y_traj, 'b--')
    
        plt.xlim(0, 20)
        plt.ylim(0, 20)
    
        plt.pause(dt)


    def transformation_matrix(self, x, y, theta):
        return np.array([
            [np.cos(theta), -np.sin(theta), x],
            [np.sin(theta), np.cos(theta), y],
            [0, 0, 1]
            ])
          
    def show_image(self, states):
        img = mpimg.imread('r2d2.png')
         
        rotation_in_degrees = math.degrees(states[2])
        rotated_img = ndimage.rotate(img, rotation_in_degrees, cval = 255)
        plt.imshow(rotated_img, extent=(states[0] - 0.25, states[0] + 0.25, states[1] - 0.25, states[1] + 0.25))
          
    def plot_pose(self, pose, color):
        plt.plot(pose[0],pose[1], color)
        plt.arrow(pose[0], pose[1], np.cos(pose[2]), np.sin(pose[2]), color=color, width=0.005)
          
        
    def animate(self, states_history, goal, path, maps, controls):
        plt.cla()
        plt.plot(states_history[0][:], states_history[1][:], ".b")
        plt.plot(path[0], path[1], '.r')
        self.plot_pose(states_history[:,-1], 'b')
        self.plot_pose(goal, 'r')
        maps.plot_map()
        self.plot_car(states_history[0][-1], states_history[1][-1], states_history[2][-1], controls[1], cabcolor="-r", truckcolor="-k")
#        self.show_image(states_history[:,-1])
        #plt.axis([min(min(states_history[0,:]), goal[0]) - 2.0, max(max(states_history[0,:]), goal[0])+ 2.0, min(min(states_history[1,:]), goal[1])- 2.0, max(max(states_history[1,:]), goal[1])+ 2.0])
#        plt.axis("equal")
        plt.grid(True)
        plt.pause(0.1)
        
#            plt.arrow(states[1][0], states[2][0], np.cos(states[3][0]),
#                      np.sin(states[3][0]), color='r', width=0.01)
                  
#            plt.arrow(x_goal, y_goal, np.cos(theta_goal),
#                      np.sin(theta_goal), color='g', width=0.1)
                  
#            plot_vehicle(states[0][0], states[1][0], states[2][0], states_history2[0][:], states_history2[1][:], dT, states[4][0], pid[1])

    def plot_map(x, y):
        plt.plot(x,y,".b")
        
        
    def plot_car(self,x, y, yaw, steer=0.0, cabcolor="-r", truckcolor="-k"):  # pragma: no cover
        # Vehicle parameters
        LENGTH = 4.5  # [m]
        WIDTH = 2.0  # [m]
        BACKTOWHEEL = 1.0  # [m]
        WHEEL_LEN = 0.3  # [m]
        WHEEL_WIDTH = 0.2  # [m]
        TREAD = 0.7  # [m]
        WB = 2.5 # [m]
        outline = np.array([[-BACKTOWHEEL, (LENGTH - BACKTOWHEEL), (LENGTH - BACKTOWHEEL), -BACKTOWHEEL, -BACKTOWHEEL],
                            [WIDTH / 2, WIDTH / 2, - WIDTH / 2, -WIDTH / 2, WIDTH / 2]])
    
        fr_wheel = np.array([[WHEEL_LEN, -WHEEL_LEN, -WHEEL_LEN, WHEEL_LEN, WHEEL_LEN],
                             [-WHEEL_WIDTH - TREAD, -WHEEL_WIDTH - TREAD, WHEEL_WIDTH - TREAD, WHEEL_WIDTH - TREAD, -WHEEL_WIDTH - TREAD]])
    
        rr_wheel = np.copy(fr_wheel)
    
        fl_wheel = np.copy(fr_wheel)
        fl_wheel[1, :] *= -1
        rl_wheel = np.copy(rr_wheel)
        rl_wheel[1, :] *= -1
    
        Rot1 = np.array([[math.cos(yaw), math.sin(yaw)],
                         [-math.sin(yaw), math.cos(yaw)]])
        Rot2 = np.array([[math.cos(steer), math.sin(steer)],
                         [-math.sin(steer), math.cos(steer)]])
    
        fr_wheel = (fr_wheel.T.dot(Rot2)).T
        fl_wheel = (fl_wheel.T.dot(Rot2)).T
        fr_wheel[0, :] += WB
        fl_wheel[0, :] += WB
    
        fr_wheel = (fr_wheel.T.dot(Rot1)).T
        fl_wheel = (fl_wheel.T.dot(Rot1)).T
    
        outline = (outline.T.dot(Rot1)).T
        rr_wheel = (rr_wheel.T.dot(Rot1)).T
        rl_wheel = (rl_wheel.T.dot(Rot1)).T
    
        outline[0, :] += x
        outline[1, :] += y
        fr_wheel[0, :] += x
        fr_wheel[1, :] += y
        rr_wheel[0, :] += x
        rr_wheel[1, :] += y
        fl_wheel[0, :] += x
        fl_wheel[1, :] += y
        rl_wheel[0, :] += x
        rl_wheel[1, :] += y
    
        plt.plot(np.array(outline[0, :]).flatten(),
                 np.array(outline[1, :]).flatten(), truckcolor)
        plt.plot(np.array(fr_wheel[0, :]).flatten(),
                 np.array(fr_wheel[1, :]).flatten(), truckcolor)
        plt.plot(np.array(rr_wheel[0, :]).flatten(),
                 np.array(rr_wheel[1, :]).flatten(), truckcolor)
        plt.plot(np.array(fl_wheel[0, :]).flatten(),
                 np.array(fl_wheel[1, :]).flatten(), truckcolor)
        plt.plot(np.array(rl_wheel[0, :]).flatten(),
                 np.array(rl_wheel[1, :]).flatten(), truckcolor)
        plt.plot(x, y, "*")