# -*- coding: utf-8 -*-
"""
Created on Mon Jan 24 16:25:59 2022

@author: jnguy
"""

import matplotlib.pyplot as plt

class PlotSituation():
    """
    sanity check plots
    """
    def __init__(self, grid, obstacle_list):
        
        self.grid = grid
        self.grid_size = [grid.x_size+1, grid.y_size+1, grid.z_size] 
        self.obstacle_list = obstacle_list
        
    def __set_axis(self):
        fig = plt.figure()
        ax = Axes3D(fig)
        ax.set_xlim([-1, self.grid_size[0]])
        ax.set_ylim([-1, self.grid_size[1]])
        ax.set_zlim([0,  self.grid_size[2]])
        return fig,ax
    
    def __plot_static_obstacles(self, ax):
        for obstacle in self.obstacle_list:
            obstacle_z_list = obstacle.z_list
            for obstacle_z in obstacle_z_list:
               ax.scatter(obstacle.x, obstacle.y, obstacle_z, color='red')
        
    def plot_config_space(self):
        """just plotting the configuration space"""
        fig, ax = self.__set_axis()
        
        self.__plot_static_obstacles(ax)                

        #lets plot regions in here too
        color_list = ['g', 'c', 'm', 'b']
        line_styles = ['-', '--', '-.', ':']
        cluster_dict = annotated_map.cluster_dict
        for i, (cluster_key, cluster) in enumerate(cluster_dict.items()):            
            for j, (entrance_key, entrance) in enumerate(cluster.mapped_entrances.items()):
                if entrance:
                    x_val = [x[0] for x in entrance]
                    y_val = [y[1] for y in entrance]
                    z_val = [z[2] for z in entrance]
                    ax.plot(x_val, y_val, z_val[0], color = color_list[i], linestyle=line_styles[j],
                            label=(cluster_key,entrance_key))
                    #ax.scatter(x_val, y_val, z_val, color=color_list[i], marker='x')
                    
        #xticks = np.arange(0,self.grid[0])
        ax.legend()
        plt.grid()
        plt.show()
        
    def __extract_keys(self, node_keys):
        node_locations = []
        for node_key in node_keys:
            node_locations.append(eval(node_key))
            
        return node_locations
        
            
    def plot_nodes(self, graph):
        """plot connection nodes"""
        fig, ax  = self.__set_axis()
        self.__plot_static_obstacles(ax)                    
        node_dict = graph.graph
        node_keys = node_dict.keys()
        node_locations = self.__extract_keys(node_keys)
        
        #lets plot regions in here too
        color_list = ['g', 'c', 'm', 'b']
        line_styles = ['-', '--', '-.', ':']
        cluster_dict = annotated_map.cluster_dict
        color_map = {str([0,0]): color_list[0],
                     str([0,1]): color_list[1],
                     str([1,0]): color_list[2],
                     str([1,1]): color_list[3]
            }


        for node_key, node_list in node_dict.items():
            ## this plots as scatter
            # for node in node_list:
            #     if node.node_type == "INTRA":
            #         node_marker = "x"
            #     else:
            #         node_marker = "o"
            #     ax.plot(node.location[0], node.location[1], node.location[2],
            #                marker=node_marker, color=color_map[str(node.cluster_coord)])
            node_coords = []
            inter_lines = []
            for node in node_list:
                if node.node_type == "INTER":
                    node_marker = "x"
                    inter_lines.append(node.location)
                    ax.plot(node.location[0], node.location[1], node.location[2],
                                 marker=node_marker, color=color_map[str(node.cluster_coord)])
                
                node_coords.append(node.location)
                x_val = [x[0] for x in node_coords]
                y_val = [y[1] for y in node_coords]
                z_val = [z[2] for z in node_coords]
                ax.plot(x_val, y_val, z_val, color=color_map[str(node.cluster_coord)])
                
        ax.set_xlabel("x position")