# -*- coding: utf-8 -*-
"""
Created on Wed Jan 19 19:13:10 2022

@author: jnguy
"""

import unittest
import sys # added!
sys.path.append("..") # added!
from src import stupid_class
import numpy as np


class TestStupidClass(unittest.TestCase):
    
    def test_stupid_class_correct(self):
        some_val = 3
        stupidclass = stupid_class.StupidClass(some_val)
        msg = "correct"
        self.assertEquals(stupidclass.scale_size(3), 9, msg)
        
        
def compute_total_distance(path):
    """compute total sum of distance travelled from path list"""
    path_array = np.diff(np.array(path), axis=0)
    segment_distance = np.sqrt((path_array ** 2).sum(axis=1))
    
    return np.sum(segment_distance)    

if __name__=='__main__':
    unittest.main()
    #pts = [[0, 42908],[1, 3],[1, 69],[1, 11],[0, 1379963888],[0, 1309937401],[0, 1],[0, 3],[0, 3],[0, 77]]
    pts = [[0,0],[1,1], [-3,4]]
    total_dist = compute_total_distance(pts)
    # test = np.array(pts)
    # d = np.diff(test, axis=0)
    # segdists = np.sqrt((d ** 2).sum(axis=1))
    
    #coords = np.array([x[0] for x in pts])
    #y_val = np.array([y[1] for y in pts])
    
    #squared_dist = np.sum((p1-p2)**2, axis=0)
    #z_val = [z[2] for z in node_coords]
   
    #total  = np.sqrt(np.square(x - x.reshape(-1,1)) + np.square(y - y.reshape(-1,1)))    
    

    


