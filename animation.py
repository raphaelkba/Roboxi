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

                   
    def plot_pose(self, pose, color):
        self.ax.plot(pose[0],pose[1], color)
        self.ax.arrow(pose[0], pose[1], np.cos(pose[2]), np.sin(pose[2]), color=color, width=0.005)
          
        
    def animate(self, states_history, robot, path, maps):
        self.ax.cla()
        self.plot_car(robot.x, robot.y, robot.theta, robot.steering_angle, 4.5)
#        self.plot_diff_driver(robot.x, robot.y, robot.theta, robot.steering_angle, 1.0, 0.2)
        self.ax.plot(states_history[0][:], states_history[1][:], ".b")
        self.ax.plot(states_history[0][-1], states_history[1][-1], "*k")
        self.ax.plot(robot.x, robot.y, "*m")
        self.ax.plot(path[0], path[1], '.g')
        
#        self.plot_pose(states_history[:,-1], 'b')
        
        maps.plot_map(self.ax)

        self.ax.grid(True)
        plt.pause(0.001)
        self.ax.axis('equal')
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

                  
    def plot_map(x, y, axis):
        axis.plot(x, y, ".b")
        
      
    def plot_diff_driver(self, x, y, yaw, steer=0.0, robot_length=0.1, wheel_radius = 0.05): 
        # plot diff driver as a circle
        robot = mpatches.Circle((x,y), robot_length, color='red')   
        self.ax.add_patch(robot)
        # plot wheels
        wheel_1 = mpatches.FancyBboxPatch((-robot_length/2, 0), wheel_radius/2, wheel_radius,
            boxstyle=mpatches.BoxStyle("Round", pad=0.01),color='black')       
        
        wheel_2 = mpatches.FancyBboxPatch((robot_length/2, 0), wheel_radius/2, wheel_radius,
            boxstyle=mpatches.BoxStyle("Round", pad=0.01),color='black')       
        
        t1 = mpl.transforms.Affine2D().rotate_deg(math.degrees(yaw)-90.0)
        t2 = mpl.transforms.Affine2D().translate(x,y)
        t = t1 + t2 + self.ax.transData
        wheel_1.set_transform(t)
        wheel_2.set_transform(t)

        self.ax.add_patch(wheel_1)
        self.ax.add_patch(wheel_2)
        
        
        
    def plot_car(self, x, y, yaw, steer=0.0, car_length=4.5):
        # Vehicle parameters
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
