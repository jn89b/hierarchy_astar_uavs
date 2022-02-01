# -*- coding: utf-8 -*-
"""
Created on Tue Feb  1 14:42:45 2022
@author: jnguy
given x,y,z 
given collision box
generate 9 points for each range
iterate through the entire bubble to compute inflation coordinates 
return coordinate points

"""
import numpy as np
import itertools

from itertools import product

bubble = 1


# vals = list(np.arange(-bubble, bubble+1, 1))

# def inflate_location(position, bounds):
#     inflated_list = []
#     """calculate bounds"""
#     for i in bounds:
#         for j in bounds:
#             for k in bounds:
#                 new_position = [position[0]+i, position[1]+j, position[2]+k]
#                 inflated_list.append(tuple(new_position))
                
#     return inflated_list


# position = [0,0,0]

# move_list = inflate_location(position, vals)
# move_list.remove((0,0,0))

def get_moves():
    bounds = list(np.arange(-bubble, bubble+1, 1))
    move_list  = []
    """calculate bounds"""
    for i in bounds:
        for j in bounds:
            for k in bounds:
                move_list.append([i,j,k])
                
    move_list.remove([0,0,0])
    return move_list

moves = get_moves()

# test_set = set()
# position_list = [[10,10,10],[11,12,13]]

# for position in position_list:
#     inflated_locs = inflate_location(position, vals)
#     test_set.update(inflated_locs)
