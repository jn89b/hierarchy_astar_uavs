# -*- coding: utf-8 -*-
"""
Do grid search only for uavs 
should save the configurations of the uavs from hiearchy search
send this configuration to the Astar

"""

from __future__ import print_function
import numpy as np
import math as m
import random

from Astar import Astar, AstarGraph, AstarLowLevel
import itertools
from itertools import combinations, permutations, product
import time
import gc
import heapq
import pickle
from operator import add, sub

def load_pkl_map(pkl_file_name):
    """returns the annotated map from a pkl file"""
    with open(map_pkl_name, 'rb') as f:
        annotated_map  = pickle.load(f)

    return annotated_map 

if __name__=='__main__':
    ##let's just load the map to save us time
    map_pkl_name = 'map_test.pkl'
    annoted_map = load_pkl_map()
    
    