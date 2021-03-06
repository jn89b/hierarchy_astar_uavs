# -*- coding: utf-8 -*-
"""
Created on Thu Jan 27 12:17:54 2022

@author: jnguy
"""

from tracemalloc import start
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d, Axes3D 

import seaborn as sns

def save_image(image_name, fig):
    """saves image"""
    image_format = 'svg' # e.g .png, .svg, etc.
    # image_name = 'myimage.svfg'
    
    fig.savefig('images/'+image_name+'.svg', format=image_format, dpi=1200)

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
        ax = plt.axes(projection="3d")
        #add offsets
        ax.set_xlim([-1,100])
        ax.set_ylim([-1,100])
        ax.set_zlim([0,50])
        # ax.set_xlim([-1, self.grid_size[0]+1])
        # ax.set_ylim([-1, self.grid_size[1]+1])
        # ax.set_zlim([-1,  self.grid_size[2]+1])
        return fig,ax
    
    
    def __set_2d_axis(self):
        fig = plt.figure()
        ax = plt.axes()
        ax.set_xlim([-1,100])
        ax.set_ylim([-1,100])
        
        return fig,ax
        
    def __plot_static_obstacles(self, ax):
        for obstacle in self.obstacle_list:
            #obstacle_z_list = obstacle.z_list
            #for obstacle_z in obstacle_z_list:
            ax.scatter(obstacle[0], obstacle[1], obstacle[2], color='red')
        
    def plot_config_space(self):
        """just plotting the configuration space"""
        fig, ax = self.__set_axis()
        
        #self.__plot_static_obstacles(ax)                

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
        #ax.legend()
        plt.grid()
        plt.show()
        save_image('Regions', fig)
        
    def __extract_keys(self, node_keys):
        node_locations = []
        for node_key in node_keys:
            node_locations.append(eval(node_key))
            
        return node_locations
            
    def plot_reservation(self,reservation_table):
        fig, ax = self.__set_axis()
        for val in reservation_table:
            ax.scatter3D(val[0], val[1], val[2])
    
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
                        
                # x_val = [x[0] for x in node_coords]
                # y_val = [y[1] for y in node_coords]
                # z_val = [z[2] for z in node_coords]
                # ax.plot(x_val, y_val, z_val, color=color_map[str(node.cluster_coord)])
                #ax.plot(x_val, y_val, z_val, color=color_list[0])
                
       # ax.set_xlabel("x position")
        
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

    def plot_inter_nodes(self,graph, fig=None, ax=None):
        if fig == None and ax == None:
            fig, ax  = self.__set_axis()
        
        else:
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
                        
                    
        save_image('Inter Nodes', fig)
        # def save_image(image_name, fig):
        #     """saves image"""
        #     image_format = 'svg' # e.g .png, .svg, etc.
        #     # image_name = 'myimage.svfg'
            
        #     fig.savefig('images/'+image_name+'.svg', format=image_format, dpi=1200)
   
    def plot_abstract_path(self, path_list, graph, color, filename):
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
        save_image(filename, fig)
   
    def plot_overall_paths(self, overall_path, graph, color,filename, uav_key=None):
        """plots the abstract from the waypoints assigned
        show the regions and entryways I have to go through"""
        fig ,ax = self.__set_axis()
            
        #self.plot_inter_nodes(graph, ax)
        # self.__plot_static_obstacles(ax)
        #self.plot_inter_nodes(graph, fig=fig, ax=ax)
        cluster_dict = self.grid.cluster_dict
        

        
        #lets plot regions in here too
        line_styles = ['-', '--', '-.', ':']

        if uav_key !=None:
            color_list = uav_key
        else:
            color_list = ['g', 'c', 'm', 'b']
        
        #plot start and stop points
        for i,path_list in enumerate(overall_path):
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
            ax.plot(x_val, y_val, z_val, color=color_list[i],fillstyle='none')
            
        
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


        # for node_key, node_list in node_dict.items():
        #     node_coords = []
        #     inter_lines = []
        #     for node in node_list:
        #         node_coords.append(node.location)
        #         if node.node_type == "INTER":
        #             node_marker = "x"
        #             inter_lines.append(node.location)
        #             ax.plot(node.location[0], node.location[1], node.location[2],
        #                           marker=node_marker, color=color_map[str(node.cluster_coord)])                     
        save_image(filename, fig)
            
    def plot_2d_paths(self, overall_path, graph, color,filename, uav_key=None):
        """"""
        fig,ax = self.__set_2d_axis()

        #self.plot_inter_nodes(graph, ax)
        # self.__plot_static_obstacles(ax)
        #self.plot_inter_nodes(graph, fig=fig, ax=ax)
        cluster_dict = self.grid.cluster_dict
        
        #lets plot regions in here too
        line_styles = ['-', '--', '-.', ':']

        if uav_key !=None:
            color_list = uav_key
        else:
            color_list = ['g', 'c', 'm', 'b']
        
        #plot start and stop points
        for i,path_list in enumerate(overall_path):
            path_coords = []
            if isinstance(path_list, int):
                continue
            
            for path in path_list:

                start_end_size = 50
                start_points = path_list[0]
                end_points = path_list[-1]
                ax.scatter(start_points[0], start_points[1], color=color_list[i], marker='o',
                            s=start_end_size)
                ax.scatter(end_points[0], end_points[1], color=color_list[i], marker='^',
                            s=start_end_size)
                path_coords.append(path)
            
            
            x_val = [x[0] for x in path_coords]
            y_val = [y[1] for y in path_coords]
            ax.plot(x_val, y_val, color=color_list[i],marker='o',fillstyle='none')

            
            plt.xticks(fontsize=16)
            plt.yticks(fontsize=16)
        
    def plot_start_goal(self, overall_path, graph, color,filename):
        """plots the abstract from the waypoints assigned
        show the regions and entryways I have to go through"""
        fig ,ax = self.__set_axis()

        #self.plot_inter_nodes(graph, ax)
        # self.__plot_static_obstacles(ax)
        #self.plot_inter_nodes(graph, fig=fig, ax=ax)
        cluster_dict = self.grid.cluster_dict
        
        #lets plot regions in here too
        color_list = ['g', 'c', 'm', 'b']
        line_styles = ['-', '--', '-.', ':']
        
        
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
            ax.plot(x_val, y_val, z_val, color=str(color), fillstyle='none')
            
        
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
        save_image(filename, fig)
            
class Plotter():
    """generic class to plot stuff as I go on this semester"""
    def __init__(self):
        """styles can be dark,etc"""
        self.style = sns.set_style("darkgrid")
        self.fontsize = 16
    
    def plot_basic_line(self, x_vals, y_vals, title_name,  x_label, y_label):
        """plot axis labels"""
        fig = plt.figure()
        plt.plot(x_vals, y_vals)
        
        plt.title(title_name, fontsize=self.fontsize)
        plt.xlabel(x_label, fontsize=self.fontsize)
        plt.ylabel(y_label, fontsize=self.fontsize)
        plt.tight_layout()
        
    def plot_multiple_lines(self, x_list, y_list, line_labels, title_name, x_label, y_label):
        """plot multiple from x list and y list has line labels to refer to the line 
        this assumes that you have the same x axis, which you probably should have as well
        as the same units for comparison for your y axis"""
        fig = plt.figure()
        
        for i,y_vals in enumerate(y_list):
            plt.plot(x_list, y_vals, label=line_labels[i])

        plt.title(title_name, fontsize=self.fontsize)
        plt.xlabel(x_label, fontsize=self.fontsize)
        plt.ylabel(y_label, fontsize=self.fontsize)
        plt.legend()
        plt.tight_layout()
        
    def plot_multiple_response(self, x_list, y_list, line_labels, title_name, x_label, y_label):
        """plot multiple from x list and y list has line labels to refer to the line 
        this assumes that you have the same x axis, which you probably should have as well
        as the same units for comparison for your y axis"""
        fig = plt.figure()
        color_pallete = sns.color_palette("rocket", n_colors=len(y_list))
        print("color is ", color_pallete)
        
        for i, (x_vals,y_vals,line_names) in enumerate(zip(x_list, y_list, line_labels)):
            plt.plot(x_vals, y_vals, label=line_names, color = color_pallete[i])

        plt.title(title_name, fontsize=self.fontsize)
        plt.xlabel(x_label, fontsize=self.fontsize)
        plt.ylabel(y_label, fontsize=self.fontsize)
        plt.legend()
        plt.tight_layout()
        
    def plot_subplots(self, num_subplots, num_cols, x_list, y_list):
        """plot a bunch of subplots to the system """
        # https://stackoverflow.com/questions/12319796/dynamically-add-create-subplots-in-matplotlib
        # Subplots are organized in a Rows x Cols Grid

        # Tot and Cols are known
        Tot = num_subplots
        Cols = num_cols

        # Compute Rows required
        Rows = Tot // Cols 
        Rows += Tot % Cols
        
        # Create a Position index        
        Position = range(1,Tot + 1)
    
        # Create main figure
        fig = plt.figure()
        for k in range(Tot):

            # add every single subplot to the figure with a for loop
            
            for x_vals,y_vals in zip(x_list, y_list):
                ax = fig.add_subplot(Rows,Cols,Position[k])
                for x, y in zip(x_vals, y_vals):
                    ax.plot(x, y)

        plt.show()

        
    



