# -*- coding: utf-8 -*-
"""
Created on Mon Jan 31 21:34:22 2022

@author: jnguy
"""
import pickle
import glob
import os

"""testing how to load a bunch of pkl files"""

def load_pkl_file(pkl_file_name):
    """opens pkl file name"""
    with open(pkl_file_name, 'rb') as f:
        return pickle.load(f)


def get_all_pkl_names(path_directory):
    """get all csv files based on some path directory"""
    return glob.glob(path_directory + "/*.pkl")


if __name__=='__main__':
    
    folder_name = "logs"
    path = os.getcwd() + "/"+ folder_name
    
    # # #get all csvs and compile to dataframe
    pkl_file_names = get_all_pkl_names(path)
    
    pkl_list = []
    
    for pkl in pkl_file_names:
        pkl_list.append(load_pkl_file(pkl))
        

    for dictionary in pkl_list:
        print("number of uavs", len(dictionary['time_list']))
        print("time of simulation", sum(dictionary['time_list']))
        print(dictionary['success'])
    