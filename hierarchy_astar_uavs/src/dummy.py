# -*- coding: utf-8 -*-
"""
Created on Sun Nov 28 19:19:04 2021

@author: jnguy
"""

import multiprocessing
import numpy as np


def has_left_zone(zone_coords, uav_coords):
    """check if uav has left zone in the long and lat distance"""
    zone_coords = np.array(zone_coords)
    uav_coords = np.array(uav_coords)
    dist = abs(np.sqrt((zone_coords[0]- uav_coords[0])**2+(zone_coords[1]- uav_coords[1])**2))
    if dist >= 10:
        return True
    else:
        return False

def add_distance(some_location):
    some_location[0] +=1 
    some_location[1] +=1
    print("new location is", some_location)
    return some_location

def observe_something(some_location, zone_area, some_list):
    print("checking", some_location)
    
    while has_left_zone(some_location, some_location) == False:
        if has_left_zone(zone_area, some_location):
            print("has left zone", some_location)
            return 
        else:
            print("nope")
            add_distance(some_location)

def multi_process(test_list, landing_area):
    processes = []
    
    for i, location in enumerate(test_list[:]):
        p = multiprocessing.Process(observe_something(location, landing_area[i], test_list))
        print("starting thread", location)
        p.start()
        processes.append(p)
    for p in processes:
        #wait until we finish before continuing 
        p.join()
    
if __name__ == '__main__':
    test_list = [[0,0], [2,2]]
    landing_area = [[0,0], [2,2]]
    multi_process(test_list, landing_area)

    
    