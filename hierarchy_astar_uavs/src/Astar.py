# -*- coding: utf-8 -*-
"""
Created on Thu Nov 11 18:04:48 2021

@author: jnguy
"""

#!/usr/bin/env python
# -*- coding: utf-8 -*- 
from __future__ import print_function
#from bson.objectid import ObjectId

import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d, Axes3D 

from scipy import spatial
from queue import PriorityQueue

from datetime import *

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

class Astar():
    """Astar"""
    def __init__(self, grid, obst_dict, obs_list,start, goal, height, ground):
        self.grid = grid
        self.grid_z, self.grid_x, self.grid_y  = grid.shape
        self.start = [int(i) for i in start]
        self.goal = goal
        self.collision_bubble = 1.0
        self.height_boundary = height
        self.ground_boundary = ground
        
        self.obst_dict = obst_dict
        self.obstacle_list = obs_list

        self.openset = PriorityQueue() # priority queue
        self.closedset = {}
        print("start and end", start, goal)

    def add_offset():
        """this is stupid add offset to grid"""
        
    def map_to_grid(self,node_position):
        """get into z,x,y"""
        return [node_position[2], node_position[0], node_position[1]]

    def is_collision(self,distance):
        """check if there is a collision if so return True"""
        if distance <= self.collision_bubble:
            return True
    
    def find_closest_obstacle(self, obstacles, current_position):
        """find closest obstacle from obstacle list, wrt current position"""
        tree = spatial.KDTree(obstacles)
        dist, obst_index = tree.query(current_position)   
        
        return dist, obst_index
    
    def init_node(self):
        start_node = Node(None,tuple(self.start))
        start_node.g = start_node.h = start_node.f = 0
        self.openset.put((start_node.f, start_node))
        self.end_node = Node(None, tuple(self.goal))
        self.end_node.g = self.end_node.h = self.end_node.f = 0

    def is_move_valid(self, node_position):
        """check if move made is valid if so then return True"""     
        print(node_position)
        if (node_position[0] > (self.grid_x-1) or 
            (node_position[0] < 0)):
            print("x")
            return False
        
        if (node_position[1] > (self.grid_y-1) or 
            (node_position[1] < 0)):
            print("y")
            return False
        
        if node_position[2] > self.height_boundary-1:
            #print(node_position)
            print("z")
            return False
        
        if node_position[2] < self.ground_boundary:
            print("ground")
            return False
        
    
    def is_target_close(self, position, goal):
        """refactor this, just have distance as input"""
        """check if we are close to target if so we remove the penalty heuristic for 
        flying high or low"""
        distance = self.compute_euclidean(position,goal)
        if distance <= 1.5:
            return True

    def compute_euclidean(self,position, goal):
        """compute euclidiean with position and goal as 3 vector component"""
        distance =  math.sqrt(((position[0] - goal.position[0]) ** 2) + 
                        ((position[1] - goal.position[1]) ** 2) +
                        ((position[2] - goal.position[2]) ** 2))
        
        return distance

    #This function return the path of the search
    def return_path(self, current_node, grid):
        
        path = []
        no_rows = self.grid_x
        no_columns = self.grid_y
        # here we create the initialized result maze with -1 in every position
        result = [[-1 for i in range(no_columns)] for j in range(no_rows)]
        current = current_node
        
        while current is not None:
            path.append(current.position)
            current = current.parent
        # Return reversed path as we need to show from start to end path
        path = path[::-1]
        start_value = 0
        # we update the path of start to end found by A-star serch with every step incremented by 1
        for i in range(len(path)):
            result[path[i][0]][path[i][1]] = start_value
            start_value += 1
        #print("path is found", path)
        return path
    
    def main(self):
        ss = 1
        move  =  [[ss, 0, 0 ], # go forward
                  [ 0, -ss, 0], # go left
                  [ -ss, 0 , 0], # go backward
                  [ 0, ss, 0 ], #go right
                  [ss, ss, 0 ], #go forward right
                  [ss, -ss, 0], #go forward left
                  [-ss, ss, 0 ], #go back right
                  [-ss, -ss, 0], #go back left
                  [ 0, ss , ss], #go up z 
                  [ 0, ss, -ss]] # go down z
        print("starting")
        self.init_node()
        count = 0 

        """main implementation"""
        while not self.openset.empty():
            if self.openset.empty():
                print("No more moves")
                return self.closedset
            
            count = count + 1
            if count >= 10000:
                print("iterations too much")
                return self.closedset

            #pop node off from priority queue and add into closedset
            cost,current_node = self.openset.get()
            self.closedset[current_node.position] = current_node
               
            #check if we hit the goal 
            if current_node.position == self.end_node.position:
                print("Goal reached", current_node.position)
                path = self.return_path(current_node, self.grid)
                #print("success!", count)
                return path
  
            #move generation
            children = []
            for new_position in move:
                
                node_position = (current_node.position[0] + new_position[0],\
                                 current_node.position[1] + new_position[1],\
                                     current_node.position[2] + new_position[2])

                #CHECK WITHIN BOUNDS
                if self.is_move_valid(node_position) == False:
                    #print("out of bounds", node_position)
                    continue
        
                #CHECK IF WALKABLE
                # if node_position in self.obst_dict:
                #     continue
                # print(node_position)
                if self.grid[node_position[2], node_position[0], node_position[1]] == 1:
                    continue
                
                
                #COLLISION CHECK
                dist, obst_index = self.find_closest_obstacle(self.obstacle_list, node_position)
                #print("checking", self.obstacle_list[obst_index])
                if self.is_collision(dist):
                    #print("collision")
                    continue
                
                #create new node
                new_node = Node(current_node, node_position)
                
                # put to possible paths
                children.append(new_node)
                    
            #check each children 
            for child in children:
                #check if children is already visited
                if child.position in self.closedset:
                    continue
                
                if abs(current_node.position[2] - child.position[2]) == 1:
                    penalty = 1.15
                    #print("penalty", penalty)
                else:
                    penalty = 1                                                 
                
                """Heuristic costs calculated here, this is using eucledian distance"""
                #print("child.position", child.position)
                if self.is_target_close(current_node.position, self.end_node):
                    #print("current_node", current_node.position)
                    child.g = current_node.g + 1
                    child.h = self.compute_euclidean(child.position, self.end_node)
                    dynamic_weight = 0.5
                    child.f = child.g + (child.h *penalty*dynamic_weight)
                    #print(child.f)
                else:
                    #print(current_node.g)
                    child.g = current_node.g + 1
                    dynamic_weight = 1.0
                    child.h = self.compute_euclidean(child.position, self.end_node)
                    child.f = child.g + (child.h *penalty*dynamic_weight)
                
                #add to open set
                #print("putting in", child)
                self.openset.put((child.f, child))