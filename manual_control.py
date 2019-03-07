"""
Roboxi - Robotics Toolbox simulator
Manual Control

Author: Raphael Kusumoto Barbosa de Almeida
E-Mail : raphael_kba@hotmail.com
"""

import numpy as np

class ManualControl():
    def __init__(self):
        self.acceleration = 0.0
        self.steering_vel = 0.0
        
        
    def get_controls(self, fig, states, dT):
        def on_key(event):
            keyboard = event.key
#            print('you pressed', event.key, event.xdata, event.ydata)
            if keyboard == 'up':
#                print("up")
                self.acceleration = 0.1
            elif keyboard == 'down':
#                print("down")
                self.acceleration = -0.1
            elif keyboard == 'left':
#                print("left")
                self.steering_vel = 1.5
            elif keyboard == 'right':
#                print("right")
                self.steering_vel = -1.5
            else:
                self.steering_vel = 0.0
                self.acceleration = 0.0
            
        cid = fig.canvas.mpl_connect('key_press_event', on_key)
        
#        self.steering_vel = (self.steering_vel - states[4,0])/dT
        return np.array([self.acceleration, self.steering_vel])
    