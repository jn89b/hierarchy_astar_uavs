
# -*- coding: utf-8 -*-
"""
Created on Wed Oct 20 16:14:14 2021

@author: jn89b

Astar 
take in grid size
take in collision objects 
take in x amount of drones
"""
from queue import PriorityQueue
from scipy import spatial

import numpy as np
import collections
import heapq
import numpy as np 
import matplotlib.pyplot as plt
import math as m
import re,seaborn as sns
from matplotlib.animation import FuncAnimation, PillowWriter, FFMpegWriter
import time

import os
import glob
import csv
import pandas as pd
from IPython import display

class PreFlightPlanner():
    """Look at unique permutations for zone assignments
    and run order
    if run order creates and """
    def __init__(self):
        pass

class UAV():
    """this is a fake uav has an id and position"""
    def __init__(self, id_name , position, index, goal):
        self.id = id_name
        self.starting_position = position 
        self.goalpoint = goal
        self.zone_index = index
        self.path = None
        self.offset_path = None

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

    def __init__(self, grid, obs_list,start, goal, col_bubble, weight_factor):
        self.grid = grid
        self.start = start
        self.goal = goal
        print("starting", start, goal)
        self.collision_bubble = col_bubble
        self.weight_factor = weight_factor
        self.height_boundary = 20
        self.ground_boundary = 5
        
        self.obstacle_list = obs_list

        self.openset = PriorityQueue() # priority queue
        self.closedset = {}
        #self.openset = []

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
        #self.openset.append(start_node)
        self.end_node = Node(None, tuple(self.goal))
        self.end_node.g = self.end_node.h = self.end_node.f = 0

    def is_move_valid(self, node_position):
        """check if move made is valid if so then return True"""
        if (node_position[0] > (len(self.grid) - 1) or 
            node_position[0] < 0 or 
            node_position[1] > (len(self.grid)-1) or 
            node_position[1] < 0 or
            node_position[2] > self.height_boundary or
            node_position[2] < self.ground_boundary ):
            return False
    
    def is_target_close(self, position, goal):
        """check if we are close to target if so we remove the penalty heuristic for 
        flying high or low"""
        distance = self.compute_euclidean(position, goal)
        
        if distance <= 1.5:
            return True
        
    #This function return the path of the search
    def return_path(self,current_node, grid):
        path = []
        no_rows = len(grid)
        no_columns = len(grid)
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
            
        return path
    
    
    def compute_euclidean(self,position, goal):
        """compute euclidean distance"""
        distance =  m.sqrt(((position[0] - goal.position[0]) ** 2) + 
                           ((position[1] - goal.position[1]) ** 2) +
                           ((position[2] - goal.position[2]) ** 2))
        
        return distance
    
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
        
        self.init_node()
        
        count = 0 
        """main implementation"""
        while not self.openset.empty():
        #while len(self.openset) > 0:
            count = count + 1
            #print(count)
            if count >= 4000:
                print("iterations too much")
                return None, count, self.closedset 
            
            if self.openset.empty():
                print("No more moves")
            
            #pop node off from priority queue and add into closedset
            cost,current_node = self.openset.get()
            self.closedset[current_node.position] = current_node
               
            #check if we hit the goal 
            if current_node.position == self.end_node.position:
                #print("Goal reached", current_node.position)
                path = self.return_path(current_node, grid)
                print("success!", count)
                return path, count, self.closedset
  
            #move generation
            children = []
            for new_position in move:

                # Get node position
                #print(current_node.position)
                
                node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1],  current_node.position[2] + new_position[2])
                #print(node_position)
                # Make sure within range (check if within maze boundary)
                if self.is_move_valid(node_position) == False:
                    #print("move is invalid")
                    continue
        
                # Make sure walkable terrain
                if self.grid[node_position] != 0:
                    #print("not walkable")
                    continue
                
                #check collision bubble here
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
                    #print("Exists", child.position)
                    continue
                
                if abs(current_node.position[2] - child.position[2]) == 1:
                    penalty = 1.15
                    #print("penalty", penalty)
                else:
                    penalty = 1                                                 
                
                """Heuristic costs calculated here, this is using eucledian distance"""
                #print("child.position", child.position)
                if self.is_target_close(current_node.position, self.end_node):
                    print("current_node", current_node.position)
                    print("target is close", current_node.position)
                    cost = self.compute_euclidean(current_node.position, child)
                    child.g = current_node.g + 1
                    child.h = self.compute_euclidean(child.position, self.end_node)
                    dynamic_weight = 0.75
                    child.f = child.g + (child.h *penalty*dynamic_weight)
                    print(child.f)
                else:
                    cost = self.compute_euclidean(current_node.position, child)
                    #print(current_node.g)
                    child.g = current_node.g + 1
                    #dynamic_weight = 15
                    child.h = self.compute_euclidean(child.position, self.end_node)
                    child.f = child.g + (child.h *penalty*self.weight_factor)
                #print(child.f)
                
                #add to open set
                #print("putting in", child)
                self.openset.put((child.f, child))


#%% general function setup
def generate_grid(grid_row, grid_col, grid_height):
    """generate grids"""
    grid = []
    grid = np.zeros((grid_height, grid_row, grid_col))
    
    return grid

def add_obstacles(grid, obstacle_list):
    """"add obstacles to grid location"""
    for obstacle in obstacle_list:
        (grid[obstacle[2],obstacle[0], obstacle[1]]) = 1
        
    return obstacle_list

def compute_actual_euclidean(position, goal):
    distance =  (((position[0] - goal.position[0]) ** 2) + 
                       ((position[1] - goal.position[1]) ** 2) +
                       ((position[2] - goal.position[2]) ** 2))**(1/2)
    
    return distance
    
#plot_path(grid_z, grid_x , grid_y, uav_0.path, obstacle_list, uav_0.goalpoint)
def plot_path(grid_z, grid_x, grid_y, waypoint_list, obstacles, goal, plot_obst):
    fig = plt.figure(figsize=(8,8),dpi=100)
    ax = plt.axes(projection='3d')
    ax.set_xlim([-1, grid_x])
    ax.set_ylim([-1, grid_y])
    ax.set_zlim([-1, 30])

    if plot_obst == True:
        for obstacle in obstacles:
            print(obstacle)
            if (obstacle[0], obstacle[1],5) in landing_zones:            
                ax.scatter(obstacle[0],obstacle[1], obstacle[2], color='green')
            else:
                ax.scatter(obstacle[0],obstacle[1], obstacle[2], color='red')
    
    ax.scatter(goal[0], goal[1], goal[2], color='red', marker="+")       
    #plot waypoints
    x = [x[0] for x in waypoint_list]
    y = [y[1] for y in waypoint_list]
    z = [z[2] for z in waypoint_list]
    ax.plot3D(x,y,z, 'bo', linestyle="-")
    

    plt.grid()
    plt.show()


def return_other_zones(zones, index):
    """return all other zones not assigned to uav to make as a no fly zone"""
    copy = zones
    copy.pop(index)
    return copy

def return_other_uavs(uavs, uav_index):
    """return all other zones not assigned to uav to make as a no fly zone"""
    copy = uavs
    copy.pop(uav_index)
    return copy


def get_offset_wp(uav_path, home_base_loc):
    array = np.array(uav_path)
    result = (array[:,0] - homebase_loc[0], array[:,1] - homebase_loc[1], array[:,2])
    x = result[0]
    y = result[1]
    z = result[2]
    offset_wp = [list(coords) for coords in zip(x,y,z) ]
    
    return offset_wp


def simulate_results(uav_list, uav_loc, col_bubble, weight_factor):
    """pass"""
    waypoint_dict = {}
    obstacle_dict = {}
    search_dict = {}
    path_obst = []
    count_list = []

    for idx, uav in enumerate(uav_list):
        print(idx)
        zone_obstacles = []
        if idx == 0:
            other_zones = return_other_zones(landing_zones[:], uav.zone_index)
            for other_zone in other_zones:
                x = other_zone[0]
                y = other_zone[1]
                for z in range(25):
                    zone_obstacles.append((x,y,z))
            new_obstacle = obstacle_list + zone_obstacles + return_other_uavs(uav_loc[:], idx)
        else:
            #append more than path to uav
            zone_obstacles = []
            other_zones = return_other_zones(landing_zones[:], uav.zone_index)
            for other_zone in other_zones:
                x = other_zone[0]
                y = other_zone[1]
                for z in range(25):
                    zone_obstacles.append((x,y,z))
                            
            path_obst.append(uav_list[idx-1].path)
            if path_obst != None:
                flat_list = [item for sublist in path_obst for item in sublist]
                new_obstacle = obstacle_list + zone_obstacles + return_other_uavs(uav_loc[:], idx) + flat_list 
            
        grid_copy = grid.copy()
        new_obstacle = add_obstacles(grid_copy, new_obstacle)
        astar = Astar(grid_copy, new_obstacle,  uav.starting_position, uav.goalpoint, col_bubble, weight_factor)
        uav.path, count, visited = astar.main()
        obstacle_dict[uav.id] = new_obstacle
        waypoint_dict[uav.id] = uav.path
        search_dict[uav.id] = visited
        count_list.append(count)
        
    return waypoint_dict, obstacle_dict, search_dict, count_list

def return_vector_list(waypoints):
    """return vector list"""
    x = [x[0] for x in waypoints]
    y = [y[1] for y in waypoints]
    z = [z[2] for z in waypoints]
    
    return np.array((x,y,z), dtype=float)

def update_lines(num, data, line):
    line.set_data(data[:2, :num])
    line.set_3d_properties(data[2, :num])
    
    # line2.set_data(data2[:2, :num])
    # line2.set_3d_properties(data2[2, :num])
    
    line.set_label('Visited Nodes') # set the label and draw the legend
    # line2.set_label('Visited Path') 
    #plt.legend(bbox_to_anchor=(1.05, 0.05))
    
    return line,


def save_np_to_csv(filename, np_array):
    np.savetxt(filename+".csv", np_array, delimiter="," )
    print("saved to", filename+".csv")
    
def compile_to_single_df(csv_files):
    overall_df = pd.DataFrame()
    for i,file_directory in enumerate(csv_files):
        csv_df = pd.read_csv(file_directory)
        overall_df = overall_df.append((csv_df))
        
    return overall_df.to_numpy()

    
if __name__ == '__main__':
    plt.close('all')
    grid_z = 50 # this is probably the z axis
    grid_x = 50 # this is x
    grid_y = 50 # this is y
    grid = generate_grid(grid_z, grid_x,grid_y)
    
    static_obstacle_list = [(30,10), (10,10)]
    homebase_loc = [grid_x/2, grid_y/2]
    
    some_list = []
    for static_obstacle in static_obstacle_list:
        x = static_obstacle[0]
        y = static_obstacle[1]
        for z in range(25):
            some_list.append((x,y,z))
    
    obstacle_list = some_list
    obstacle_list = add_obstacles(grid, obstacle_list)
    
    landing_zones = [(4, 10,5), (20,30,5), (30, 20, 5), (30, 30, 5)] 
    
    # #uav0
    uav_0 = UAV("uav0", [3, 9, 15], 0, landing_zones[0])
    uav_1 = UAV("uav1", [13, 0, 9], 2, landing_zones[2])
    uav_2 = UAV("uav2", [49, 20, 12], 3, landing_zones[3])
    uav_3 = UAV("uav3", [49, 16, 13], 1, landing_zones[1])          
    
    uav_list = [uav_0, uav_1, uav_2] #uav_3]
    uav_loc = [uav_0.starting_position, uav_1.starting_position, uav_2.starting_position]#, uav_3.starting_position]
    
    heuristic = 1.5
    waypoint_dict, obstacle_dict, search_space_dict, count_list = simulate_results(uav_list, uav_loc, 4, heuristic)
    waypoint_dict_2, obstacle_dict_2, search_space_dict_2 , count_list_2 = simulate_results(uav_list, uav_loc, 4, 15)
    
    plot_path(grid_z, grid_x, grid_y, waypoint_dict['uav0'], obstacle_dict['uav0'], uav_0.goalpoint, True)
    plot_path(grid_z, grid_x, grid_y, waypoint_dict['uav1'], obstacle_dict['uav1'], uav_1.goalpoint, True)
    
    # #%% Open all CVS in the current directory
    # cwd = os.getcwd()
    # extension = 'csv'
    # #os.chdir(path)
    # results = glob.glob('*.{}'.format(extension))
    # df_list  = []
    # for result in results:
    #     print(result)
    #     df_list.append(pd.read_csv(result, quoting=csv.QUOTE_NONE, error_bad_lines=True))
    # #df = compile_to_single_df(result[0])
    

    #%% Testing out plots
    # uavname = 'uav3'
    # zone_num = 1    
    # N = 100
    
    # ## First Subplot -----
    # fig = plt.figure(figsize=(12,8))
    # #ax1
    # ax = fig.add_subplot(121, projection='3d')
    # #ax2 = fig.add_subplot(122, projection='3d')
    # #set up plots
    # ax.set_xlim([-1, grid_x])
    # ax.set_ylim([-1, grid_y])
    # #ax.set_zlim([-1, 30])
    
    # # ax2.set_xlim([-1, grid_x])
    # # ax2.set_ylim([-1, grid_y])
    # # ax2.set_zlim([-1, 30])
            
    # #plot search space
    # # search_vector = return_vector_list(search_space_dict[uavname])
    # # search_vector_2 = return_vector_list(search_space_dict_2[uavname])
    
    # search_vector = np.transpose(df_list[2].to_numpy())
    # search_vector_2 = np.transpose(df_list[3].to_numpy())
    
    # path_1 = np.transpose(df_list[0].to_numpy())
    # path_2 = np.transpose(df_list[1].to_numpy())
    
    # # search_vector = df[0,:]
    # # search_vector = df[1,:]
    
    # #line animation plot
    # #line, = ax.plot(search_vector[0, 0:1], search_vector[1, 0:1], search_vector[2, 0:1], label = 'Visited Nodes')
    # # line2, = ax2.plot(search_vector_2[0, 0:1], search_vector_2[1, 0:1], search_vector_2[2, 0:1], label = 'Visited Path')
    
    # ax.scatter(search_vector[0, :], search_vector[1, :], search_vector[2, :], label = 'Visited Nodes')
    # ax.plot3D(path_1[0],path_1[1],path_1[2], 'orange', linestyle="-", linewidth=4.0, label='Actual Path')
    # # ani = FuncAnimation(fig, update_lines, fargs=(search_vector, line), \
    # #                     interval= 20, blit=True, repeat = True, frames=search_vector.shape[1]+1)
    
    # ax = fig.add_subplot(122, projection='3d')
    # ax.set_xlim([-1, grid_x])
    # ax.set_ylim([-1, grid_y])
    # ax.scatter(search_vector_2[0, :], search_vector_2[1, :], search_vector_2[2, :], label = 'Visited Nodes')
    # ax.plot3D(path_2[0],path_2[1],path_2[2], 'orange', linestyle="-", linewidth=4.0, label='Actual Path')
    # plt.grid()
    # plt.tight_layout()
    # plt.show()
    
    # fig.tight_layout()
    # #plot goal point
    # # ax.scatter(landing_zones[zone_num][0], landing_zones[zone_num][1],landing_zones[zone_num][2],color='red', s= 10.0)    
    # # ax2.scatter(landing_zones[zone_num][0], landing_zones[zone_num][1],landing_zones[zone_num][2],color='red', s= 10.0)    
    
    # # x,y,z = return_vector_list(waypoint_dict[uavname])
    # # x2,y2,z2 = return_vector_list(waypoint_dict_2[uavname])
    
    
    # #ax.plot3D(path_2[0], path_2[1], path_2[2], 'orange', linestyle="-", linewidth=4.0)
    
    # #titles 
    # #heuristic = "Weighted Heuristic: %.2f, Visited Nodes: %2d" % (15, search_vector.shape[1])
    # #ax.set_title(heuristic, y=1.0)
    # #heuristic2 = "Weighted Heuristic: %2d, Visited States: %2d" % (15, search_vector_2.shape[1])
    # #ax2.set_title(heuristic2)
    
    # #legend set up
    # handles,labels = ax.get_legend_handles_labels()
    # fig.legend(handles, labels, loc='lower center')
    
    #anim = FuncAnimation(fig, animate, frames = data_points, init_func = init, interval = 20, blit = True)
    #writervideo = FFMpegWriter(fps=search_vector.shape[1]/0.8)
    #ani.save("low_heuristic.mp4", writer=writervideo)
    
    #plt.close('all')
    
    
    #video = ani.to_html5_video()
    #html = display.HTML(video)
    #display.display(html)
    ##save to csvs
    # save_np_to_csv("search 1", np.transpose(search_vector))
    # save_np_to_csv("search 2", np.transpose(search_vector_2))
    # save_np_to_csv("path 1", np.transpose([x,y,z]))
    # save_np_to_csv("path 2", np.transpose([x2,y2,z2]))
    # f = os.getcwd() 
    # writervideo = FuncAnimation.FFMpegWriter(fps=60) 
    ##anim.save(f, writer=writervideo)
    #ani.save('test.mp4', writer=writervideo) #writer=PillowWriter(fps=30))

    
