# -*- coding: utf-8 -*-
"""
Created on Sat Mar 19 14:03:46 2022

@author: jnguy

Original position at 0,0,50
Goal point at 6,0,50

"""
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
from IPython.display import HTML # Animation on jupyter lab 
from matplotlib.animation import PillowWriter # For GIF animation 
#####Data Generation####
import pandas as pd
import pickle 


x = []
y = []
z = []



def load_pickle(pkl_file):
    """load pkl file"""
    with open(pkl_file, 'rb') as f:
        path = pickle.load(f)
        
    return path
     

class AnimateSearchSpace():
    """
    needs the following inputs:
        self.method_name = type(str) the name of the path finding algorithm using
        self.path_method = list of 3 consisting of [path, iterations, and dictionary of visited]
        
    """
    def __init__(self, method_name, path_method, method_position, start_pos, goal_pos):
        self.x = []
        self.y = []
        self.z = []
        
        self.start_pos = start_pos 
        self.goal_pos = goal_pos
        self.method_name = method_name
        self.path_method = path_method
        self.method_position = method_position

    def __set_size_params(self,x_bounds, y_bounds, z_bounds):
        self.fig = plt.figure(figsize=(8, 8))
        self.ax = self.fig.add_subplot(111, projection="3d")
        self.ax.set_xlim3d(x_bounds[0], x_bounds[1])
        self.ax.set_ylim3d(y_bounds[0], y_bounds[1])
        self.ax.set_zlim3d(z_bounds[0], z_bounds[1])
        self.title = self.ax.set_title(self.method_name)

    def update_graph(self,num):
        if num == len(self.method_position)-1:
            print("yes")
            x_list = [position[0] for position in self.path_method[0]]
            y_list = [position[1] for position in self.path_method[0]]
            z_list = [position[2] for position in self.path_method[0]]
            for path in self.path_method[0]:    
                self.ax.plot(x_list, y_list, z_list, color='blue',linewidth=5)
                self.title.set_text('{}, Number of Visited Nodes={}, Total Number of Iterations {}'.format(
                    self.method_name, num, self.path_method[1]))
        else:
            data = self.method_position[num]
            print("num", num)
            self.x.append(data[0])
            self.y.append(data[1])
            self.z.append(data[2])
            self.graph._offsets3d = (self.x, self.y, self.z)
            self.title.set_text('{}, Number of Visited Nodes={}'.format(self.method_name, num))
        
        return self.graph,
    
    def plot_start_goal(self):
        self.ax.scatter(self.start_pos[0], self.start_pos[1], self.start_pos[2], color='green', s=60, marker='x')
        self.ax.scatter(self.goal_pos[0], self.goal_pos[1], self.goal_pos[2], color='red', s=60, marker='x')
    
    def animate_plot(self, x_bounds, y_bounds, z_bounds):
        """animate the 3d plot with specificiations of the bounds of the grid to plot"""
        marker_size = 80
        self.__set_size_params(x_bounds, y_bounds, z_bounds)
        
        
        self.graph = self.ax.scatter(self.method_position[0], self.method_position[1][0], self.method_position[2][0], color='orange')
        self.plot_start_goal()
    
        self.ani = animation.FuncAnimation(self.fig, self.update_graph,frames=len(self.method_position), 
            interval=10, blit=False, repeat=False)

        plt.show()
        

start_pos = [0,0,50]
goal_pos = [6,0,50]

plt.close('all')
x_bounds = [0,8]
y_bounds = [0,8]
z_bounds = [45,55]

djikstra = load_pickle('djikstra_path.pkl')
djikstra_position = [list(v.position) for (k,v) in djikstra[2].items()]
djikstra_search = AnimateSearchSpace(start_pos=start_pos, goal_pos=goal_pos, method_name='Djikstra', path_method=djikstra, method_position=djikstra_position)
djikstra_search.animate_plot(x_bounds=x_bounds, y_bounds=y_bounds, z_bounds=z_bounds)


astarreg = load_pickle('astarreg.pkl')
astarreg_position = [list(v.position) for (k,v) in astarreg[2].items()]
astarreg_search = AnimateSearchSpace(start_pos=start_pos, goal_pos=goal_pos, method_name='Astar', path_method=astarreg, method_position=astarreg_position)
astarreg_search.animate_plot(x_bounds, y_bounds, z_bounds)
