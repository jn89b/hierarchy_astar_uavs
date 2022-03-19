# -*- coding: utf-8 -*-
"""
Created on Fri Mar 18 20:30:19 2022

@author: jnguy
"""

from Astar import Astar, AstarGraph, AstarLowLevel, Djikstras,AstarReg

import pickle
import hierarchal_astar

if __name__ == '__main__':
    
    
    ## PARAMS
    x_size = 100
    y_size = 100
    z_size = 75
    
    z_obs_height = 1
    num_clusters = 4
    
    load_map = False
    load_graph = False
    save_information = True
    
    map_pkl_name = 'map_test.pkl'
    graph_pkl_name = 'test.pkl'
    

    if load_map == True:
        with open(map_pkl_name, 'rb') as f:
            annotated_map  = pickle.load(f)
    else:
        ##### CONFIGURATION SPACE
        annotated_map= hierarchal_astar.build_map(3, x_size, y_size, z_size, z_obs_height, num_clusters, 10)
            
    ####-------- GRAPH 
    """I need to cache this to a database and query it to reduce start up costs
    I should save the information about the ostacles as well or maybe annoted map"""
    if load_graph == True:
        random_obstacles  = annotated_map._static_obstacles
        #obst_coords = get_obstacle_coordinates(random_obstacles)
        graph = hierarchal_astar.Graph(annotated_map, load_graph, graph_pkl_name)
    else:    
        random_obstacles = hierarchal_astar.generate_random_obstacles(1, x_size, z_obs_height)
        graph = hierarchal_astar.Graph(annotated_map, load_graph, graph_pkl_name)
        graph.build_graph()    
        graph.build_intra_edges()        
        hierarchal_astar.set_obstacles_to_grid(grid=annotated_map, obstacle_list=random_obstacles)
        

    test_grid = annotated_map._Map__grid
    astar_test_graph = graph.graph
    obst_coords = annotated_map._Map__obstacles #i don't know why this is private
    
    ##begin adding uavs in to the sys btem
    obst_set = set(tuple(x) for x in obst_coords)
    x_bounds = [1,x_size-1]
    y_bounds = [1,y_size-1]
    z_bounds = [5,z_size-1]
    
    n_uav_list = [1]
    n_simulations = 1
    #n_uavs = 10
    spacing = 10
    
    reservation_table = set()
    col_bubble = 4
    weighted_h = 1
    
    test_grid = annotated_map._Map__grid
        
    
    djikstra = Djikstras(
        test_grid, reservation_table, obst_coords,
        [0,0,50], [6,0,50], col_bubble, weighted_h
        )
    
    djikstra_path = djikstra.main()

    astarreg = AstarReg(
        test_grid, reservation_table, obst_coords,
        [0,0,50], [6,0,50], col_bubble, weighted_h
        )
        
    astarreg_path = astarreg.main()

    #%% pickle_save
    with open('djikstra_path.pkl', 'wb') as f:
        pickle.dump(djikstra_path , f)
    
    #%% load pkl file
    with open('astarreg.pkl', 'wb') as f:
        pickle.dump(astarreg_path , f)