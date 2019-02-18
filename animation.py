"""
Robotic Animation Class
Author: Raphael Kusumoto Barbosa de Almeida
E-Mail : raphael_kba@hotmail.com
"""
import numpy as np
import math
import matplotlib.pyplot as plt
import matplotlib as mpl
import matplotlib.image as mpimg
from matplotlib import transforms
from scipy import ndimage
import matplotlib.transforms as mtransforms
import matplotlib.patches as mpatches



class animation():
    
    def __init__(self):
        self.fig, self.ax = plt.subplots()


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
        self.ax.plot(pose[0],pose[1], color)
        self.ax.arrow(pose[0], pose[1], np.cos(pose[2]), np.sin(pose[2]), color=color, width=0.005)
          
        
    def animate(self, states_history, goal, path, maps, controls):
        self.ax.cla()
        self.plot_car(states_history[0][-1], states_history[1][-1], states_history[2][-1], controls[1], 2.5)

        self.ax.plot(states_history[0][:], states_history[1][:], ".b")
        self.ax.plot(path[0], path[1], '.g')
        
        self.plot_pose(states_history[:,-1], 'b')
        self.plot_pose(goal, 'r')
        
        maps.plot_map(self.ax)

        self.ax.grid(True)
        plt.pause(0.1)
        self.ax.axis('equal')
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

                  
    def plot_map(x, y, axis):
        axis.plot(x,y,".b")
        
        
    def plot_car(self,x, y, yaw, steer=0.0, carlength=4.5):
        # Vehicle parameters
        car_length = carlength  # [m]
        car_width = car_length/2.25  # [m]
        backwheel = car_length/4.5   # [m]
        wheel_length = backwheel*0.5  # [m]
        wheel_width = backwheel*0.3 # [m]
        wheel_distance = backwheel*0.7  # [m]
        wheel_offset = backwheel*2.5 # [m]
        
        # plot car rectangle
        car = mpatches.FancyBboxPatch((-backwheel,-car_width/2), car_width, car_length,
            boxstyle=mpatches.BoxStyle("Round", pad=0.01),color='red')       
        t1 = mpl.transforms.Affine2D().rotate_deg(math.degrees(yaw)-90.0)
        t2 = mpl.transforms.Affine2D().translate(x,y)
        t = t1 + t2 + self.ax.transData
        car.set_transform(t)
        self.ax.add_patch(car)
                
        # plot back wheels
        wheel_1 = mpatches.FancyBboxPatch((-wheel_distance- wheel_width/2,-wheel_length/2), wheel_width, wheel_length,
            boxstyle=mpatches.BoxStyle("Round", pad=0.01),color='black')       
        
        wheel_2 = mpatches.FancyBboxPatch((wheel_distance - wheel_width/2,-wheel_length/2), wheel_width, wheel_length,
            boxstyle=mpatches.BoxStyle("Round", pad=0.01),color='black')       
        
        wheel_1.set_transform(t)
        wheel_2.set_transform(t)

        self.ax.add_patch(wheel_1)
        self.ax.add_patch(wheel_2)
        
        # plot front wheels
        wheel_3 = mpatches.FancyBboxPatch((-wheel_distance - wheel_width/2,wheel_offset - wheel_length/2), wheel_width, wheel_length,
            boxstyle=mpatches.BoxStyle("Round", pad=0.01),color='black')         
        
        wheel_4 = mpatches.FancyBboxPatch((wheel_distance - wheel_width/2,wheel_offset - wheel_length/2), wheel_width, wheel_length,
            boxstyle=mpatches.BoxStyle("Round", pad=0.01),color='black')    
        tl = mpl.transforms.Affine2D().rotate_around(-wheel_distance, wheel_offset, steer)
        tr = mpl.transforms.Affine2D().rotate_around(wheel_distance, wheel_offset, steer)        
        
        tl_ =  tl + t1 + t2 + self.ax.transData
        tr_ =  tr + t1 + t2 + self.ax.transData
    
        wheel_3.set_transform(tl_)
        wheel_4.set_transform(tr_)
        
        self.ax.add_patch(wheel_3)
        self.ax.add_patch(wheel_4)
