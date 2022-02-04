# -*- coding: utf-8 -*-
"""
Created on Mon Jan 31 21:34:22 2022
@author: jnguy


What do I want to see?
Get the big picture/snap shot first:
    Classify by number of UAVs in the simulation:
        =percent success
        -search space iterations
        -time it took to find solutions
    Solution:
        Have a dataframe for N UAVS:

SAVE AS AN OVERALL PKL FRAME JUST APPEND TO THE VALUES
    
Evaluate different UAVs percentage of success:
    Why does it drop off?
    Is there a mathematical model to represent this based on:
        grid space
        number of uavs
        number of nodes
        the heuristics used
        the collision bubble formulated
    What about the quality of solutions:
        Using straight line distance... for now 
        Can compare to the abstract paths as well 
    What is the storage capacity?
        Biggest thing is the reservation table -> 
        how does it correlate to finding solution?
        
"""
import pickle
import glob
import os
import pandas as pd
from collections import defaultdict
import matplotlib.pyplot as plt
import numpy as np
from scipy import stats

class DataParser():
    """class used to extract data its more of a utility class """
    
    def __init__(self):
        pass
    
    def load_pkl_file(self,pkl_file_name):
        """opens pkl file name"""
        with open(pkl_file_name, 'rb') as f:
            return pickle.load(f)
        
    def get_all_pkl_names(self,path_directory):
        """get all csv files based on some path directory"""
        return glob.glob(path_directory + "/*.pkl")
    
    def get_filename(self,filename_dir):
        """return base file names of csv and removes the directory path"""
        return os.path.basename(os.path.normpath(filename_dir))
    
class NUAVDataframe():
    """dataframe that holds provides overall infor"""
    def __init__(self):
        self.df = pd.DataFrame(data = None, 
                               columns = ['Timestamp','Label','Occurance', 'Confidence'],
                              )
    
def init_uavs_dictionary(start, stop, steps):
    """returns a dictionary of keys for number of uavs """
    n_uavs_keys = [i for i in range(start,stop+steps, steps)]
   
    new_dict = defaultdict(list)
    for n_uav in n_uavs_keys:
        new_dict[n_uav] = None
    
    return new_dict
    
def init_nuav_df():
    """initializes an overall dataframe"""
    df = pd.DataFrame(data = None, 
                      columns = ['SimNum','Heuristic','Bubble','Total Time',
                                 'Iter Time', 'Success'],
                      )
    
    return df

def insert_df_to_dict(uav_dict):
    """inserts a dataframe into the dictionary"""
    for n_uav in uav_dict:
        df = init_nuav_df()
        uav_dict[n_uav] = df
    
def insert_overall_dict():
    """adds an overall dictionary inside key of input dictionary"""
    overall_keys = ['SimNum','Heuristic','Bubble','Total Time',
               'Iter Time', 'Success']
    
    new_dict = defaultdict(list)
    for n_uav in overall_keys:
        new_dict[n_uav] = []
    
    return new_dict

def insert_into_overall_dict(pkl_dict):
    """inserts information inside the overall_dict"""
    
#%% Loading the dictionaries 
if __name__=='__main__':
    
    dataparser = DataParser()
    
    pkl_dirs = []
    
    for i in range(1,8):
        folder_name = "logs/sim_"+str(i)
        path = os.getcwd() + "/"+ folder_name
        pkl_file_dirnames = dataparser.get_all_pkl_names(path)
        
        pkl_dirs.extend(pkl_file_dirnames)
        
    pkl_list = []
    pkl_filenames = []
    for pkl in pkl_dirs:
        pkl_list.append(dataparser.load_pkl_file(pkl))
        pkl_filenames.append(dataparser.get_filename(pkl))
    

#%% Splitting up the dictionaries based on UAVs, for now..

    DICT_KEYS = ["abstract_paths", 
                 "goal_list", 
                 "iter_list", 
                 "success", 
                 "time_list"]
    
    keys = list(pkl_list[0].keys())
    
    n_uav_dictionary = init_uavs_dictionary(10, 120, 10)
    for key in n_uav_dictionary:
        n_uav_dictionary[key] = insert_overall_dict()
    
    for pkl,file_name in zip(pkl_list, pkl_filenames):
        num_uavs = len(pkl["overall_paths"])
        n_uav_dictionary[num_uavs]["Bubble"] = 4
        n_uav_dictionary[num_uavs]["Heuristic"] = 10
        n_uav_dictionary[num_uavs]["SimNum"].append(file_name) 
        n_uav_dictionary[num_uavs]["Success"].append(pkl['success'])
        n_uav_dictionary[num_uavs]["Total Time"].append(sum(pkl["time_list"]))
        n_uav_dictionary[num_uavs]["Iter Time"].append(sum(pkl["iter_list"]))
    

#%% Convert overall dictionaries to dataframes for easier formatting and parsing
    df_list = []
    
    for key,overall_dict in n_uav_dictionary.items():
        overall_df = pd.DataFrame.from_dict(overall_dict)
        df_list.append(overall_df)

    """I should save in a pkl file to save time"""

#%% Is there a correlation between distance and iteration time?
"""
Also need to include uavs spawn as obstacles for the initial start up

If i have a long path set at the end of the queue or middle, does this cause 
the large run time and space complexity??

"""

#%% Testing out PLOTS
    
    plt.close('all')
    #https://stackoverflow.com/questions/66446681/python-use-bins-from-one-sns-histplot-for-another-extract-bin-information
    import seaborn as sns
    
    from itertools import cycle
    def plot_subplots(df_list, column_name,x_label, main_title):
        n_rows = 4
        n_cols = 3 
        c = 1
        fig, axes = plt.subplots(n_rows,n_cols,sharex='col', sharey=False)        
        axes = axes.flat 
        numbers = np.arange(10,100+20,10)
        for i, df in enumerate(df_list):
            sns.histplot(df[column_name], bins='auto', color='steelblue', ax=axes[i], 
                         stat='density', kde=True)
            #sns.kdeplot(data=df[column_name], ax=axes[i],shade=True)
            
            axes[i].set_title(str((i+1)*10) + " for " + str(len(df)) + " simulations")
            axes[i].set_xlabel(x_label)
            
    
        
        fig.suptitle(main_title)

    plot_subplots(df_list, "Success", "Success Rate", "Overall Success")
    plot_subplots(df_list, "Total Time", "Time Seconds", "Time to Plan Path")
    plot_subplots(df_list, "Iter Time", "Number of Iterations", "Total Number of Iterations")

    



    
