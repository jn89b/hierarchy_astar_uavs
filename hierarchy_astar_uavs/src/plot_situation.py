# -*- coding: utf-8 -*-
"""
Created on Thu Jan 27 12:17:54 2022

@author: jnguy
"""

from tracemalloc import start
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d, Axes3D 

class PlotSituation():
    """
    sanity check plots
    """
    def __init__(self, grid, obstacle_list):
        
        self.grid = grid
        self.grid_size = [grid.x_size+1, grid.y_size+1, grid.z_size] 
        self.obstacle_list = obstacle_list

        ## COLOR PARAMS
        self.color_list = ['g', 'c', 'm', 'b']
        self.color_map = {str([0,0]): self.color_list[0],
                     str([0,1]): self.color_list[1],
                     str([1,0]): self.color_list[2],
                     str([1,1]): self.color_list[3]
            }
        
    def __set_axis(self):
        fig = plt.figure()
        ax = Axes3D(fig)
        #add offsets
        ax.set_xlim([-1, self.grid_size[0]+1])
        ax.set_ylim([-1, self.grid_size[1]+1])
        ax.set_zlim([-1,  self.grid_size[2]+1])
        return fig,ax
    
    def __plot_static_obstacles(self, ax):
        for obstacle in self.obstacle_list:
            #obstacle_z_list = obstacle.z_list
            #for obstacle_z in obstacle_z_list:
            ax.scatter(obstacle[0], obstacle[1], obstacle[2], color='red')
        
    def plot_config_space(self):
        """just plotting the configuration space"""
        fig, ax = self.__set_axis()
        
        self.__plot_static_obstacles(ax)                

        #lets plot regions in here too
        color_list = ['g', 'c', 'm', 'b']
        line_styles = ['-', '--', '-.', ':']
        cluster_dict = self.grid.cluster_dict
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
        cluster_dict = self.grid.cluster_dict
        color_map = {str([0,0]): color_list[0],
                     str([0,1]): color_list[1],
                     str([1,0]): color_list[2],
                     str([1,1]): color_list[3]
            }


        for node_key, node_list in node_dict.items():
            node_coords = []
            inter_lines = []
            for node in node_list:
                node_coords.append(node.location)
                if node.node_type == "INTER":
                    node_marker = "x"
                    inter_lines.append(node.location)
                    ax.plot(node.location[0], node.location[1], node.location[2],
                                  marker=node_marker, color=color_map[str(node.cluster_coord)])
                        
                x_val = [x[0] for x in node_coords]
                y_val = [y[1] for y in node_coords]
                z_val = [z[2] for z in node_coords]
                ax.plot(x_val, y_val, z_val, color=color_map[str(node.cluster_coord)])
                #ax.plot(x_val, y_val, z_val, color=color_list[0])
                
        ax.set_xlabel("x position")
        
    def plot_start_end(self, graph, start_position):
        fig, ax  = self.__set_axis()
        self.__plot_static_obstacles(ax)                    
        node_dict = graph.graph
        node_keys = node_dict.keys()
        node_locations = self.__extract_keys(node_keys)
        
        #lets plot regions in here too
        color_list = ['g', 'c', 'm', 'b']
        line_styles = ['-', '--', '-.', ':']
        cluster_dict = self.annotated_map.cluster_dict
        color_map = {str([0,0]): color_list[0],
                     str([0,1]): color_list[1],
                     str([1,0]): color_list[2],
                     str([1,1]): color_list[3]
            }
        
        for node_key, node_list in node_dict.items():
            node_coords = []
            inter_lines = []
            for node in node_list:
                node_coords.append(node.location)
                if node.node_type == "INTER":
                    node_marker = "x"
                    inter_lines.append(node.location)
                    ax.plot(node.location[0], node.location[1], node.location[2],
                                  marker=node_marker, color=color_map[str(node.cluster_coord)])
                    
    def plot_quadrant(self, graph, coordinates):
        """plot the quadrant"""
        color_list = ['g', 'c', 'm', 'b']
        fig,ax = self.__set_axis()
        self.__plot_static_obstacles(ax)
        x_val = [x[0] for x in coordinates]
        y_val = [y[1] for y in coordinates]
        z_val = [z[2] for z in coordinates]
        ax.plot(x_val, y_val, z_val, color=color_list[0])

    def plot_inter_nodes(self,graph):
        fig, ax  = self.__set_axis()
        node_dict = graph.graph
        node_keys = node_dict.keys()
        node_locations = self.__extract_keys(node_keys)

        for node_key, node_list in node_dict.items():
            node_coords = []
            inter_lines = []
            for node in node_list:
                node_coords.append(node.location)
                if node.node_type == "INTER":
                    node_marker = "x"
                    inter_lines.append(node.location)
                    ax.plot(node.location[0], node.location[1], node.location[2],
                                    marker=node_marker, color=self.color_map[str(node.cluster_coord)])
   
    def plot_abstract_path(self, path_list, graph, color):
        """plots the abstract from the waypoints assigned
        show the regions and entryways I have to go through"""
        fig ,ax = self.__set_axis()

        #self.plot_inter_nodes(graph, ax)
        self.__plot_static_obstacles(ax)
        #plot start and stop points
        start_end_size = 50
        start_points = path_list[0]
        end_points = path_list[-1]
        ax.scatter3D(start_points[0], start_points[1], start_points[2], color="cyan", marker='o',
                    s=start_end_size)
        ax.scatter3D(end_points[0], end_points[1], end_points[2], color="green", marker='^',
                    s=start_end_size)

        path_coords = []
        for path in path_list:
            path_coords.append(path)
        x_val = [x[0] for x in path_coords]
        y_val = [y[1] for y in path_coords]
        z_val = [z[2] for z in path_coords]
        ax.plot(x_val, y_val, z_val, color=str(color))
        
                
   
    def plot_overall_paths(self, overall_path, graph, color):
        """plots the abstract from the waypoints assigned
        show the regions and entryways I have to go through"""
        fig ,ax = self.__set_axis()

        #self.plot_inter_nodes(graph, ax)
        self.__plot_static_obstacles(ax)
        #plot start and stop points

        for path_list in overall_path:
            path_coords = []
            if isinstance(path_list, int):
                continue
            for path in path_list:

                start_end_size = 50
                start_points = path_list[0]
                end_points = path_list[-1]
                ax.scatter3D(start_points[0], start_points[1], start_points[2], color="cyan", marker='o',
                            s=start_end_size)
                ax.scatter3D(end_points[0], end_points[1], end_points[2], color="green", marker='^',
                            s=start_end_size)
                path_coords.append(path)
            x_val = [x[0] for x in path_coords]
            y_val = [y[1] for y in path_coords]
            z_val = [z[2] for z in path_coords]
            ax.plot(x_val, y_val, z_val, color=str(color))
        
    



