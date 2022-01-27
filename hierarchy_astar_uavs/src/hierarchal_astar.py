# -*- coding: utf-8 -*-
"""
Graphs have edges and nodes

Grids have a component of a graph

"""

#!/usr/bin/env python
# -*- coding: utf-8 -*- 
from __future__ import print_function
import numpy as np
import math as m
import matplotlib.pyplot as plt
import random

from Astar import Astar
from mpl_toolkits.mplot3d import axes3d, Axes3D 
import itertools
from itertools import combinations, permutations, product
import time
import gc
import heapq


class Map():
    """
    overall search space, hence why I call it a Map
    """
    def __init__(self, z_size, x_size, y_size, obstacle_list):        
        self.x_size = x_size
        self.y_size = y_size
        self.z_size = z_size
        self.offset_val = 1 # this is stupid but the amount of step sizes in the grid
        self.__grid = self.__generate_grid(z_size, x_size, y_size)
        self.__obstacles = obstacle_list
        
        
        self._static_obstacles = {}
        self.__dynamic_obstacles = {}
        self.entrance_dict = {}
        self.cluster_dict = {}
        #should rename to move cluster
        self.__move_cluster_dict = {'left_entrance': [-1,0],
                     'right_entrance':[1,0],
                     'lower_entrance': [0,-1],
                     'upper_entrance': [0,1]}
        
    def get_grid(self):
        """return grid"""
        return self.__grid
    
    def __generate_grid(self, z_size, x_size, y_size):
        """returns the grid size of the grid"""
        return np.zeros((z_size, x_size, y_size))
    
    def get_obstacle_list(self):
        return self.__obstacles

    def __insert_into_static_obstacle_db(self, obstacle):
        """appends object into static obstal"""
        self._static_obstacles[obstacle.x, obstacle.y] = obstacle

    def __check_valid_cluster_size(self, n_clusters):
        """check if the number of clusters to make equal squares 
        are valid"""
        if (n_clusters % 2) != 0:
            print("you need an even number")
            return False
        if ((self.x_size*self.y_size) % n_clusters) != 0:
            print("can't do this")
            return False
        else:
            return True

    def __get_cluster_bounds(self,index, step_size):
        """get min and max bounds of the cluster"""
        #print(index, index+step_size)
        return [index, index+step_size-self.offset_val] 

    def __compute_cluster_size(self, n_clusters):
        """compute the cluster size dimension of the grid"""
        return (self.x_size**2/n_clusters)
    
    def __get_cluster_edges(self, max_bound, cluster_space):
        """return cluster edges from the cluster space:
            left edge is [x_lower, length of cluster]
            right edge is [x_upper , length of cluster]
            size should be a 2D vector [max_bound , height of z] 
        """
        upper_edge = cluster_space[:, :, max_bound-self.offset_val]
        left_edge = cluster_space[:, 0,:]
        right_edge = cluster_space[:, max_bound-self.offset_val, :]
        lower_edge = cluster_space[:, :, 0]
        cluster_edges = {'left_edge': left_edge,
                         'right_edge': right_edge,
                         'upper_edge': upper_edge,
                         'lower_edge': lower_edge} 
        
        return cluster_edges
    
    def __get_opposite_direction(self, entrance_key):
        """return the index of the opposite direction"""
        self.opposite_direction = {'left_entrance': 'right_entrance',
                     'right_entrance':'left_entrance',
                     'lower_entrance': 'upper_entrance',
                     'upper_entrance': 'lower_entrance'}
        
        return self.opposite_direction[entrance_key]
    
    def get_adjacent_clusters(self, entrance_key, cluster_pos):
        """return adjacent cluster based on the current cluster position and the 
        move made"""
        move = self.__move_cluster_dict[entrance_key]
        adjacent_cluster_position= [cluster_pos[0] + move[0], cluster_pos[1] + move[1]]        
        opposite_entrance = self.__get_opposite_direction(entrance_key) 
        
        return adjacent_cluster_position, opposite_entrance
    
    def place_static_obstacle(self, obstacle):
        """place obstacles inside the grid"""
        self.__insert_into_static_obstacle_db(obstacle)
        for obstacle_z in obstacle.z_list:
            self.__grid[obstacle_z, obstacle.x, obstacle.y] = 1
    
    def __check_duplicate_entrances(self, cluster_coords, adjacent_cluster_position):
        """check if we have duplicate cluster coordinates between two points"""
        paired_clusters = cluster_coords, adjacent_cluster_position
        flipped_pair = adjacent_cluster_position, cluster_coords 
        if str(paired_clusters) in self.entrance_dict:
            print("exists")
            return True
        if str(flipped_pair) in self.entrance_dict:
            print("exists")
            return True
        return False
    
    def link_entrances(self):
        """find entrances that link together"""        
        for cluster_key, cluster in self.cluster_dict.items():
            mapped_entrances = cluster.mapped_entrances
            for entrance_key, entrance_list in mapped_entrances.copy().items():
                
                adjacent_cluster_position,  adjacent_side = self.get_adjacent_clusters(entrance_key, 
                                                       cluster.cluster_coords)
                
                if self.__check_duplicate_entrances(cluster.cluster_coords, adjacent_cluster_position) == True:
                    continue

                paired_clusters = cluster.cluster_coords, adjacent_cluster_position
                connections = [entrance_list, self.cluster_dict[str(adjacent_cluster_position)].mapped_entrances[adjacent_side],
                               cluster.cluster_coords, adjacent_cluster_position] 
                self.entrance_dict[str(paired_clusters)] = connections
                
    def prune_entrances(self):
        """remove outer edges of the spaces"""        
        max_len = m.sqrt(len(self.cluster_list))
        for cluster_key, cluster in self.cluster_dict.items():
            cluster_pos = cluster.cluster_coords
            
            for i, (entrance_key, move) in enumerate(self.__move_cluster_dict.items()):
                position = [cluster_pos[0] + move[0], cluster_pos[1] + move[1]]
                    
                if position[0] < 0 :
                    del cluster.mapped_entrances[entrance_key]
                
                if position[0] > max_len - 1:
                    del cluster.mapped_entrances[entrance_key]
                
                if position[1] < 0:
                    del cluster.mapped_entrances[entrance_key]
                
                if position[1] > max_len -1:
                    del cluster.mapped_entrances[entrance_key]
                    
    def reduce_entryways(self, n_numbers):
        """reduce the number of entryways, this is kind of stupid but for now
        set it to a hard number?"""
        for cluster, cluster_vals in self.cluster_dict.items():
            for entrance_side, entrances in cluster_vals.mapped_entrances.items():
                if n_numbers == 0:
                    continue
                for i, coordinates in enumerate(entrances[:]):
                    if (i % n_numbers) != 0:
                        #print("coordinates are", i, coordinates)
                        entrances.remove(coordinates)

    def abstract_space(self, n_clusters):
        """generate clusters based on some number you want does it into 
        square pieces for this situation"""
        #check if cluster size is feasible
        if self.__check_valid_cluster_size(n_clusters) == True:
            ## cluster size is n x n
            cluster_size = self.__compute_cluster_size(n_clusters)
            step_size = int(m.sqrt(cluster_size))
            count = 0
            cluster_i = 0
            self.cluster_list = []
            for i in range(0, self.x_size, step_size):
                cluster_j = 0
                for j in range(0, self.y_size, step_size):
                    
                    x_min_max = self.__get_cluster_bounds(i,step_size) #[x_min, x_max]
                    y_min_max = self.__get_cluster_bounds(j, step_size) #[y_min, y_max]
                    cluster_name = str([cluster_i,cluster_j]) 
                    
                    #[start:stop] had to add this stupid offset to make sure we append the right values
                    cluster_space = self.__grid[:,x_min_max[0]:x_min_max[1]+ self.offset_val,
                                                y_min_max[0]:y_min_max[1]+ self.offset_val]                        
                    
                    cluster_edges = self.__get_cluster_edges(step_size,
                                                             cluster_space)
                    limits = (x_min_max, y_min_max) 
                    
                    self.cluster_dict[cluster_name] = Cluster(cluster_name,
                                                        limits,
                                                        cluster_edges,
                                                        cluster_space,
                                                        n_clusters = n_clusters,
                                                        cluster_coords = [cluster_i, cluster_j])
                    self.cluster_list.append([cluster_i,cluster_j])
                    cluster_j +=1
                    count +=1
                cluster_i +=1
            
        else:
            print("no bueno")
            
class Graph():
    def __init__(self, configuration_map):
        self.graph = {}
        self.map = configuration_map
        self.level = 1
        
    def build_graph(self):
        """build graph"""
        #this builds the inter edges
        for neighbor_clusters, vals in self.map.entrance_dict.items():
            adj_locations1 = vals[0]
            adj_locations2 = vals[1]
            c1_loc = vals[2]
            c2_loc = vals[3]
            for loc1, loc2 in zip(adj_locations1, adj_locations2):
                
                node1 = AbstractNode(loc1, c1_loc)
                node2 = AbstractNode(loc2, c2_loc)
                
                self.__add_node(node1)
                self.__add_node(node2)
                self.__add_edge(node1, node2, 1, 1, "INTER")
                
    def build_intra_edges(self):
        """build intra edges within clusters 
        number of connectsion = entrace_con + (m-1)(n) 
        where m is the number of nodes in the current location and n is the 
        number of nodes in the adjacent side"""
        for idx, (cluster, cluster_vals) in enumerate(self.map.cluster_dict.items()):
            cluster_loc = cluster_vals.cluster_coords
            inner_connections = []
            inner_sets = []
            print("working with cluster", cluster_vals.cluster_coords)
            for edge_side, location in cluster_vals.mapped_entrances.items():
                inner_connections.append(location)
                
                #get all permuations for nodes on the same side
                same_side_combos = list(combinations(location, 2))
                for same_side in same_side_combos:
                    inner_sets.append(same_side)
            
            #add all the intra nodes between the inner sides so left and right, etc
            for r in itertools.chain(product(inner_connections[0], inner_connections[1])
                                     , product(inner_connections[1], inner_connections[0])):
                inner_sets.append(r)
                
            #intra_connections_list = inner_sets
            intra_connections_list = self.__remove_coords_diff_depth(inner_sets)
            
            #this is a test to reduce the grid
            if idx == 4:
                return intra_connections_list, inner_sets
            
            #add all the intra nodes between the adjacent nodes need to do Astar to find distance
            for intra_connections in intra_connections_list:
                intra_node1 = AbstractNode(intra_connections[0], cluster_loc)
                intra_node2 = AbstractNode(intra_connections[1], cluster_loc)
                
                config_space = self.map.cluster_dict[str(cluster_loc)].cluster_space
                config_bounds = self.map.cluster_dict[str(cluster_loc)].limits
                distance = self.__search_for_distance(intra_node1, intra_node2, config_space, config_bounds)
                self.__add_edge(intra_node1, intra_node2, distance, 1, "INTRA")
                                
    def __search_for_distance(self, intra_node1, intra_node2, cluster_space, config_bounds):
        """search for distance between two intra nodes using Astar"""
        obstacle_coords = self.map.get_obstacle_list()
        
        #this is stupid I just need to apply an offset to make to the node positions
        x_bounds = [config_bounds[0][0], config_bounds[1][0]]
        x_offset = abs(x_bounds[0])
    
        y_bounds = [config_bounds[0][1], config_bounds[1][1]]
        y_offset = abs(y_bounds[0])
        
        #print("Starting and goal point is", intra_node1.location, intra_node2.location)
        start_position = [abs(intra_node1.location[0] - x_offset),
                          abs(intra_node1.location[1] - y_offset), intra_node1.location[2]]
        
        goal_position = [abs(intra_node2.location[0] - x_bounds[0]),
                          abs(intra_node2.location[1] - y_bounds[0]), intra_node2.location[2]]
                          
        #added a garbage collection to remove clutter since this function will be used 
        gc.collect()
        astar = Astar(cluster_space, obstacle_coords, start_position,
                      goal_position, self.map.z_size, 0)
    
        path_list = astar.main()
        #print("path list is", path_list[0])
        
        #return a stupid high value if I cant find a path otherwise return the actual one
        if path_list is None:
            return 1E100
        
        if isinstance(path_list[0],list):
            return self.__compute_total_distance(path_list[0])
        else:
            return 1E100
            
        
    def __compute_total_distance(self, path):
        """compute total sum of distance travelled from path list"""
        #print("path array", path)
        path_array = np.diff(np.array(path), axis=0)
        segment_distance = np.sqrt((path_array ** 2).sum(axis=1))
        return np.sum(segment_distance)    

    def __add_edge(self, node1, node2, weight, level, node_type):
        """makes edge between two nodes"""
        node1.set_cost(weight)
        node2.set_cost(weight)
        
        node1.set_node_type(node_type)
        node2.set_node_type(node_type)
        
        #check if intra node connections exists
        if not str(node1.location) in self.graph:
            self.graph[str(node1.location)] = set([node1])
            
        if not str(node2.location) in self.graph:
            self.graph[str(node2.location)] = set([node2])
        
        self.graph[str(node1.location)].add(node2)
        self.graph[str(node2.location)].add(node1)
            
    def __add_node(self, node):
        """add node to search graph"""
        if str(node.location) in self.graph:
            self.graph[str(node.location)].add(node)
        else:
            self.graph[str(node.location)] = set([node])
    
    def __remove_coords_diff_depth(self, inner_sets):
        """we don't want to connect intraedges at different heights f or now...
        space complexity will increase -> will look back at this later"""
        for i, coordinate_set in enumerate(inner_sets[:]):
            coord1 = coordinate_set[0]
            coord2 = coordinate_set[1]
            if abs(coord1[2] - coord2[2]) >=1:
                #print("removing", coordinate_set)
                inner_sets.remove(coordinate_set)
        return inner_sets
   
    def determine_cluster(self, coordinates):
        """determine which cluster location the coordinate set is located at 
        coordinates have to be x,y,z, and makes anode based on this"""
        
        #weaker precondition stronger postcondition check
        if len(coordinates) != 3:
            print("the coordinate input must be a list of [x,y,z]")
            return None
        
        for cluster, cluster_vals in graph.map.cluster_dict.items():
            #print(cluster_vals.cluster_coords)
            x_bounds, y_bounds = cluster_vals.limits
            #print(cluster_vals.limits)
            if coordinates[0] in range(x_bounds[0], x_bounds[1]+1) and coordinates[1] in range(y_bounds[0], y_bounds[1]+1):
                print("yes", cluster_vals.cluster_coords)
                node = AbstractNode(coordinates , cluster_vals.cluster_coords)        
                return node
     
    def __add_temp_edges(self, temp_node, node2, weight, level, node_type, key_name):
        """adds the temporary node and connects it with other nodes in the hashtable"""
        temp_node.set_cost(weight)
        node2.set_cost(weight)
        
        temp_node.set_node_type(node_type)
        node2.set_node_type(node_type)
        
        #check if intra node connections exists
        if not str(key_name) in self.graph:
            self.graph[str(key_name)] = set([temp_node])
            
        if not str(node2.location) in self.graph:
            self.graph[str(node2.location)] = set([node2])
        
        self.graph[str(key_name)].add(node2)
        self.graph[str(node2.location)].add(temp_node)
            
        
    def connect_to_border(self,node, key_name):
        """connect borders to the map, I should have this in the graph class but define the key value
        so set key to start and goal to make it temporary"""
        mapped_entrances_start = graph.map.cluster_dict[str(node.cluster_coord)].mapped_entrances
        for entrance, entrance_list in mapped_entrances_start.items():
            for entrance_loc in entrance_list:
                if node.location[2] != entrance_loc[2]:
                    print(node.location[2] != entrance_loc[2])
                    continue
                else:
                    config_space = self.map.cluster_dict[str(node.cluster_coord)].cluster_space
                    config_bounds = self.map.cluster_dict[str(node.cluster_coord)].limits
                    intra_node2 = AbstractNode(entrance_loc, node.cluster_coord)
                    distance = self.__search_for_distance(node, intra_node2, config_space, config_bounds)
                    #probably should refactor this 
                    self.__add_temp_edges(node, intra_node2, distance, 1, "INTRA", key_name)
        
        
    def insert_temp_nodes(self, location, level, key_name):
        """insert temp nodes into the graph takes in the AbstractNode, level, 
        and hash key name to be inserted 
        - use methods to determine cluster
        - connect to border
        - set level
        """
        temp_node = self.determine_cluster(location)
        self.connect_to_border(temp_node, key_name)
        #self.__add_temp_edges(temp_node, node2, weight, level, node_type, key_name)
        
        
class AbstractNode():
    def __init__(self, location, cluster_coord):
        self.location = location
        self.cluster_coord = cluster_coord
        self.level = None
        self.node_type = None
    
    def set_cost(self,cost):
        """this is the edge cost"""
        self.cost = cost
        
    def set_node_type(self,node_type):
        if node_type == "INTER":
            self.node_type = "INTER"
        if node_type == "INTRA":
            self.node_type = "INTRA"
                            
    def __get_comparison(self):
        return (tuple(self.location), self.node_type)
            
    def __eq__(self, other):
        return (self.location, self.node_type) == (other.location, other.node_type)
        
    def __ne__(self, other):
        return (not self.__eq__(other))
    
    def __hash__(self):
        return hash(self.__get_comparison())
    
class Cluster():
    def __init__(self, cluster_name, limits, edges, cluster_space, n_clusters,
                 cluster_coords):
        """
        entrances that link to the next cluster/graph
        obstacles within this sector
        section of the grid
        """
        self.cluster_name = cluster_name 
        self.cluster_space = cluster_space
        self.limits = limits
        self.edges = edges
        self.mapped_entrances = self.return_mapped_entrances()
        self.n_clusters = n_clusters
        self.cluster_coords = cluster_coords
        
    def __get_2d_coords(self,condition,i, x_limits, y_limits):
        """return 2d coordinates of edge based on condition"""
        if condition == "left_edge":
            return [x_limits[0], y_limits[0]+i]
    
        if condition == "right_edge":
            return [x_limits[1], y_limits[0]+i]
        
        if condition == "lower_edge":
            return [x_limits[0]+i, y_limits[0]]
        
        if condition == "upper_edge":
            return [x_limits[0]+i, y_limits[1]]
        
    def __get_entrance_coords(self, condition,edge, x_limits, y_limits):
        """return list of entrance coords"""
        edge_row_len, edge_col_len = edge.shape
        entrance_coords = []
        for i in range(edge_col_len):
            #this is a check to see if I have an obstacle along the edge or frontier of the cluster
            z_height = edge_row_len
            if edge[0,i] == 1:
                continue 
            some_coord = self.__get_2d_coords(condition, i, x_limits, y_limits)
            
            for j in range(z_height+1): #this should be mapped to some three dimensional height
                three_d = [some_coord[0], some_coord[1], j]
                entrance_coords.append(three_d)
        
        return entrance_coords
        
    def return_mapped_entrances(self):
        """return mapped locations of entrances the first column are 
        the possible entrances case switch is better for this situation
        I need to refactor this line of code
        """
        mapped_entrances = {}
        x_limits = self.limits[0]
        y_limits = self.limits[1]
        
        for edge_direction, edge in self.edges.items():
            edge_len, edge_height =  edge.shape ##row and column size
            
            if edge_direction == "left_edge":
                entrance_coords = self.__get_entrance_coords("left_edge", edge, x_limits, y_limits)
                mapped_entrances["left_entrance"] = entrance_coords
            
            if edge_direction == "right_edge":
                entrance_coords = self.__get_entrance_coords("right_edge", edge, x_limits, y_limits)
                mapped_entrances["right_entrance"] = entrance_coords
            
            if edge_direction == "lower_edge":
                entrance_coords = self.__get_entrance_coords("lower_edge", edge, x_limits, y_limits)
                mapped_entrances["lower_entrance"] = entrance_coords

            if edge_direction == "upper_edge":
                entrance_coords = self.__get_entrance_coords("upper_edge", edge, x_limits, y_limits)
                mapped_entrances["upper_entrance"] = entrance_coords

        return mapped_entrances


class Obstacle():
    """obstacle path to put into grid"""
    def __init__(self, x_loc, y_loc, z_init, z_final):
        self.x = x_loc
        self.y = y_loc  
        self.z_list = self.__generate_z_height(z_init, z_final)
        
    def __generate_z_height(self, z_init, z_final):
        """create list height of obstacle based on z init and z final
        use list comprehension later"""
        z_list = []
        ## added 1 to get it to end at the exact number
        for z in range(z_init, z_final+1):
            z_list.append(z)
            
        return z_list

def generate_random_obstacles(n_random, bound_lim, z_obs_height):
    """creating random x and y coordinates"""
    x_random = random.sample(population=range(bound_lim), k=n_random)
    y_random = random.sample(population=range(bound_lim), k=n_random)
    
    random_static_obstacles = []
    
    for x,y in zip(x_random, y_random):
        random_static_obstacles.append(Obstacle(x_loc=x, y_loc=y, z_init=0,\
                                                z_final=z_obs_height))
    
    return random_static_obstacles

def set_obstacles_to_grid(grid, obstacle_list):
    """define obstacles"""
    for obstacle in obstacle_list:
        grid.place_static_obstacle(obstacle)    


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
        cluster_dict = annotated_map.cluster_dict
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
                    
            
def get_obstacle_coordinates(obstacles):
    """return locations of obstacles"""
    obst_coords = []
    for obstacle in obstacles:
        for z in obstacle.z_list:
            obst_coords.append([obstacle.x, obstacle.y,z])
            
    return obst_coords

def test_permuations(graph):
    intra_connections = []
    test = graph.graph
    vals = test['[9, 4, 1]']
    print("location is",  [9,4,1])
    for val in vals:
        if val.node_type == "INTRA":
            print(val.location, val.node_type)
            intra_connections.append(val.location)
            
    return intra_connections

def compute_total_distance(path):
    """compute total sum of distance travelled from path list"""
    path_array = np.diff(np.array(path), axis=0)
    segment_distance = np.sqrt((path_array ** 2).sum(axis=1))
    
    return np.sum(segment_distance)    

def compute_offsets(position, config_bounds):
    """compute offsets for search graph"""
    #this is stupid I just need to apply an offset to make to the node positions
    x_bounds = [config_bounds[0][0], config_bounds[1][0]]
    x_offset = abs(x_bounds[0])

    y_bounds = [config_bounds[0][1], config_bounds[1][1]]
    y_offset = abs(y_bounds[0])
    
    new_position = [abs(position[0] - x_offset),
                      abs(position[1] - y_offset), position[2]]
    
    return [abs(position[0] - x_offset), abs(position[1] - y_offset), position[2]]


#%% Run main functions
if __name__=='__main__':
    x_size = 10
    y_size = 10
    z_size = 2
    
    z_obs_height = 0
    num_clusters = 4
    
    random_obstacles = generate_random_obstacles(n_random=2, bound_lim=x_size, z_obs_height=z_obs_height)
    annotated_map = Map(z_size, x_size, y_size, get_obstacle_coordinates(random_obstacles))
    
    set_obstacles_to_grid(grid=annotated_map, obstacle_list=random_obstacles)
    
    ##### CONFIGURATION SPACE    
    annotated_map.abstract_space(num_clusters)
    annotated_map.prune_entrances()
    annotated_map.link_entrances()
    annotated_map.reduce_entryways(2)
    
    ####-------- GRAPH
    graph = Graph(annotated_map)
    graph.build_graph()    
    connections = graph.build_intra_edges()
        
    #%% testing the search -> refactor this              
    # def connect_to_border(node):
    #     """connect borders to the map, I should have this in the graph class but define the key value
    #     so set key to start and goal to make it temporary"""
    #     mapped_entrances_start = graph.map.cluster_dict[str(node.cluster_coord)].mapped_entrances
    #     for entrance, entrance_list in mapped_entrances_start.items():
    #         for entrance_loc in entrance_list:
    #             #check if at same level
    #             if node.location[2] != entrance_loc[2]:
    #                 print(node.location[2] != entrance_loc[2])
    #                 continue
    #             else:
    #                 config_space = graph.map.cluster_dict[str(node.cluster_coord)].cluster_space
    #                 config_bounds = graph.map.cluster_dict[str(node.cluster_coord)].limits
    #                 intra_node2 = AbstractNode(entrance_loc, node.cluster_coord)
    #                 distance = graph._Graph__search_for_distance(node, intra_node2, config_space, config_bounds)
    #                 graph._Graph__add_edge(node, intra_node2, distance, 1, "INTRA")
        
    
    ## connecting the start and goal point to the abstract map
    start_location = [8,8,1]
    goal_location = [1,2,1]
    
    graph.insert_temp_nodes(start_location, 1, start_location)
    graph.insert_temp_nodes(goal_location, 1, goal_location)
    
    # start_node = graph.determine_cluster(start_location)
    # goal_node = graph.determine_cluster(goal_location)
    # graph._Graph__add_node(start_node)
    # graph._Graph__add_node(goal_node)
    # connect_to_border(start_node)
    # connect_to_border(goal_node) 
    
    
    #%% test to get sets and see if nodes and edges are connected
    #mapped_entrances_goal = graph.map.cluster_dict["end"].mapped_entrances
    start_connections = graph.graph[str(start_location)]
    goal_connections = graph.graph[str(goal_location)]
        
    #%% Astar as graph search 
    from queue import PriorityQueue
    
    def unpack_tuple_coordinates(tuple_coords):
        return [tuple_coords[0],tuple_coords[1],tuple_coords[2]]    
    
    class Node():
        """
        parent = parent of current node
        posiition = position of node right now it will be x,y coordinates
        g = cost from start to current to node
        h = heuristic 
        f = is total cost
        """
        def __init__(self, parent, position):
            self.parent = parent
            self.position = position
            
            self.g = 0
            self.h = 0
            self.f = 0
            
        def __lt__(self, other):
            return self.f < other.f
        
        # Compare nodes
        def __eq__(self, other):
            return self.position == other.position

        # Print node
        def __repr__(self):
            return ('({0},{1})'.format(self.position, self.f))

    def compute_euclidean(position, goal):
        """compute euclidean distance"""
        distance =  m.sqrt(((position[0] - goal[0]) ** 2) + 
                           ((position[1] - goal[1]) ** 2) +
                           ((position[2] - goal[2]) ** 2))
        
        return distance
        
    ##astar search
    astar_test_graph = graph.graph
    openset = PriorityQueue() # priority queue
    closed_set = {}
    
    ##init node
    start_node = Node(None,start_location)
    start_node.g = start_node.h = start_node.f = 0
    openset.put((start_node.f, start_node))
    
    end_node = Node(None, goal_location)
    end_node.g = end_node.h = end_node.f = 0
    
    print("start and goal points are", start_location, goal_location)
    count = 0
    while not openset.empty():
        if count >= 4000:
            print("failure")
            break
                
        #pop current node off
        cost,current_node = openset.get()
        
        #check if at goal if so return path
        if current_node.position == end_node.position:
            path_home = []
            print("found path", current_node)
            current = current_node 
            while current is not None:
                path_home.append(current.position)
                current = current.parent
            #reverse path
            path_home = path_home[::-1]
            print("path home is", path_home)
            break 
        
        #print("cost is", cost)
        #print("current node is", current_node.position)
        closed_set[str(current_node.position)] = current_node
                        
        #get neighbors
        current_node_position = unpack_tuple_coordinates(current_node.position)
        
        #recieve all neighbors based on the graph request
        neighbors = astar_test_graph[str(current_node_position)]
        #search through adjacent neighbors look for possible paths
        for neighbor in neighbors:            
            
            if neighbor.location == current_node_position:
                #print("neighbor duplicates", neighbor.location)
                continue
                
            #check if already in closed set
            if str(neighbor.location) in closed_set:
                #print("already in", neighbor.location)
                continue 
            
            #print(neighbor.location)
            new_node = Node(current_node, neighbor.location)
            new_node.g = current_node.g + neighbor.cost
            new_node.h = compute_euclidean(neighbor.location, goal_location)
            new_node.f = new_node.g + new_node.h 
            
            openset.put((new_node.f, new_node))
    
    
     #%% Plotting stuff
    plt.close('all')
    plt_situation = PlotSituation(annotated_map, random_obstacles)
    plt_situation.plot_config_space()
    plt_situation.plot_nodes(graph)
    
    # cluster_00 = annotated_map.cluster_dict["[0, 0]"]
    # cluster_01 = annotated_map.cluster_dict["[0, 1]"]
    # cluster_10 = annotated_map.cluster_dict["[1, 0]"]
    # cluster_11 = annotated_map.cluster_dict["[1, 1]"]


    # connections = test_permuations(graph)
    #plt_situation.plot_quadrant(graph, connections)
    
    # plot_cluster_region(cluster_0, random_obstacles)
    # plot_cluster_region(cluster_1, random_obstacles)    
    
    