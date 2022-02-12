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
    
    # https://stackoverflow.com/questions/4529815/saving-an-object-data-persistence
    def save_object(self,obj, filename):
        with open(filename, 'wb') as outp:  # Overwrites any existing file.
            pickle.dump(obj, outp, pickle.HIGHEST_PROTOCOL)
    
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

def save_image(image_name, fig):
    """saves image"""
    image_format = 'svg' # e.g .png, .svg, etc.
    # image_name = 'myimage.svfg'
    
    fig.savefig('images/'+image_name+'.svg', format=image_format, dpi=1200)
#%% Loading the dictionaries 
if __name__=='__main__':
    
    dataparser = DataParser()
    
    pkl_dirs = []
    
    for i in range(1,9):
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
    
    for i,(pkl,file_name) in enumerate(zip(pkl_list, pkl_filenames)):
        num_uavs = len(pkl["overall_paths"])
        n_uav_dictionary[num_uavs]["Bubble"] = 4
        n_uav_dictionary[num_uavs]["Heuristic"] = 10
        n_uav_dictionary[num_uavs]["SimNum"].append(file_name) 
        n_uav_dictionary[num_uavs]["Success"].append(pkl['success'])
        n_uav_dictionary[num_uavs]["Total Time"].append(sum(pkl["time_list"]))
        n_uav_dictionary[num_uavs]["Iter Time"].append(sum(pkl["iter_list"]))
        n_uav_dictionary[num_uavs]["Location"].append(i) ## refers to the list inside pkl_list
        
        
        
#%% Convert overall dictionaries to dataframes for easier formatting and parsing
    df_list = []
    
    for key,overall_dict in n_uav_dictionary.items():
        overall_df = pd.DataFrame.from_dict(overall_dict)
        df_list.append(overall_df)

#%% Testing out PLOTS
    
    plt.close('all')
    #https://stackoverflow.com/questions/66446681/python-use-bins-from-one-sns-histplot-for-another-extract-bin-information
    import seaborn as sns
    
    from itertools import cycle
    def plot_subplots(num_plots,df_list, column_name,x_label, main_title, transform_type=None):
        
        #https://stackoverflow.com/questions/66446681/python-use-bins-from-one-sns-histplot-for-another-extract-bin-information
        sns.set_style("dark")
        colors = sns.color_palette("hls", n_colors=len(df_list))
        """plots histogram"""
        n_rows = int(num_plots/2)
        n_cols = int(2) 
        c = 1
        fig, axes = plt.subplots(n_rows,n_cols,sharey=False)        
        axes = axes.flat 
        for i, df in enumerate(df_list):
            """FREQUENCY -> """
            bins = np.histogram_bin_edges(df[column_name], bins='scott')
            
            if transform_type =="log":
                sns.histplot(np.log(df[column_name]), bins=bins, color=colors[i], ax=axes[i], 
                             stat='probability', kde=False)
            if transform_type == None:
                sns.histplot(df[column_name], bins=bins, color=colors[i], ax=axes[i], 
                             stat='probability', kde=False)
                
            #sns.kdeplot(data=df[column_name], ax=axes[i],shade=True)
            axes[i].set_title(str((i+1)*10) + " for " + str(len(df)) + " simulations")
            #mids = [rect.get_x() + rect.get_width() / 2 for rect in axes[i].patches]
            #x_range = np.arange(70, 110, 10)
            #axes[i].set_xticks(x_range)
            #axes[i].set_xlabel(x_label)
            axes[i].set_ylabel("Count Percent")
        fig.suptitle(main_title)
        fig.tight_layout()
        save_image(main_title, fig)

    def super_impose_histograms():
        """want to super impose histograms into one snapshot??b"""
        
    plot_subplots(4,df_list[0:4], "Success", "Success Rate", "Overall Success")
    
    ## these two plots have skew
    #print("skewness is", df_list[0]['Total Time'].agg(['skew', 'kurtosis']).transpose())
    
    ## why do I have to log transform this guy??    
    plot_subplots(4,df_list[0:4], "Total Time", "Time Seconds", "Time to Plan Path")
    
    # whats with the iteration mini peaks?
    plot_subplots(4,df_list[0:4], "Iter Time", "Number of Iterations", "Total Number of Iterations")
    
#%% Looking at iterations logical indexing of values that are high and checking the distance
    high_iter_30_uavs = df_list[3].loc[df_list[3]["Iter Time"]>5000]
    #row_1 = high_iter_10_uavs["Index"]
    situation = pkl_list[276]
    """Evaluate iterations/uav if success"""
    failures_dict = {}
    
    
    """
    take the mean iteration count for each UAS in each simulation realm
    """
    
    for i, df in enumerate(df_list):
        print("df is", df)
        failures_dict[str((i+1)*10)] = df.loc[(df["Success"] != 1)]
    
    
#%% Is there a correlation between distance and iteration time?
    """
    Also need to include uavs spawn as obstacles for the initial start up
    
    If i have a long path set at the end of the queue or middle, does this cause 
    the large run time and space complexity??
    
    TO DO:
        In each scenario based on N uavs get the ones that have:
            large failures 
            large time to solve
            most steps
            highest distance 
            plot this relationship
            
    DF_LIST:
        Has DF for each configuration of N UAS
        Look at mean run time what is the standard deviation?
        Look at success rate what is the standard deviation?
        Look at Iteration run and standard deviation?
        LOOK FOR the failures, long times, and time count:
            Why does it happen?
            Distance?
    """
    plt.close('all')
    def get_mean_std(df, column_name):
        """returns mean and standard deviation from dataframe and column"""
        return df[column_name].mean(), df[column_name].std()
    
    
    def plot_mean_std(df_list,col_name, titlename, xlabel, ylabel):
        """plot mean and std dev"""
        fig = plt.figure()
        sns.set_style("darkgrid")
        fontsize = 16
        colors = sns.color_palette("hls", n_colors=len(df_list))
        #colors.reverse()
        for i, df in enumerate(df_list):
            mean, std = get_mean_std(df, col_name)
            plt.errorbar(int(i+1)*10, mean, std/2, linestyle='None', marker='o',
                         color=colors[i], capsize=3)
            
        x_range = np.arange(10,50,10)
        plt.xticks(x_range)
        plt.xlabel(xlabel, fontsize=fontsize)
        plt.ylabel(ylabel, fontsize=fontsize)
        plt.title(titlename,fontsize=fontsize)
        plt.tight_layout()
        
        save_image(titlename, fig)
        
        
    test = df_list[0]
    #Success
    mean_success, std_success = get_mean_std(test, 'Success')
    #Iteration time
    mean_iter, std_iter = get_mean_std(test, 'Iter Time')
    #Total time
    mean_time, std_time = get_mean_std(test, 'Total Time')
    
    plot_mean_std(df_list[0:4], "Success", "Success Rate", "Number of UAVS", "Percent Success")
    plot_mean_std(df_list[0:4], "Iter Time","Space Complexity" ,"Number of UAVS", "Number of Iterations")
    plot_mean_std(df_list[0:4], "Total Time","Time Complexity" ,"Number of UAVS", "Time (seconds)")
    
    #%% PLOT the relationship between
    """
    - Plot success rate vs solution length as scatter plot
    - Plot success rate vs time complexity in scatter plot
    - Compare distance travelled vs optimal distance 
    """
    plt.close("all")
    def plot_success_solution(df_list,col_name, titlename, xlabel):
        """plot mean and std dev"""
        fig = plt.figure()
        sns.set_style("darkgrid")
        fontsize = 16
        colors = sns.color_palette("mako", n_colors=len(df_list))
        
        colors.reverse()
        
        for i, df in enumerate(df_list):
            label = (i+1)*10
            success = df["Success"]
            time = df[col_name]
            plt.scatter(time, success,c=colors[i], label=label)
            
        plt.xlabel(xlabel, fontsize=fontsize)
        plt.ylabel("Success Rate", fontsize=fontsize)
        plt.legend()
        plt.title(titlename,fontsize=fontsize)
        plt.tight_layout()
        
    def compare(df_list,another_col_name,col_name, titlename, xlabel):
        """plot mean and std dev"""
        fig = plt.figure()
        sns.set_style("darkgrid")
        fontsize = 16
        colors = sns.color_palette("rocket", n_colors=len(df_list))
        
        colors.reverse()
        
        for i, df in enumerate(df_list):
            label = (i+1)*10
            # success = df[another_col_name]
            # time = df[col_name]
            plt.scatter(df[col_name], df[another_col_name],c=colors[i], label=label)
            
        plt.xlabel(xlabel, fontsize=fontsize)
        plt.ylabel(str(another_col_name), fontsize=fontsize)
        plt.legend()
        plt.title(titlename,fontsize=fontsize)
        plt.tight_layout()
        
    plot_success_solution(df_list[0:4],"Iter Time", "Iterations vs Success", "Iterations")
    plot_success_solution(df_list[0:4],"Total Time", "Total Time vs Success", "Time (secs)")
    compare(df_list[0:4], "Total Time", "Iter Time", "Iterations vs Time", "Iterations")
    
    #%% Compare solution path
    plt.close('all')
    
    import math
    def truncate(number, digits) -> float:
        stepper = 10.0 ** digits
        return math.trunc(stepper * number) / stepper

    def compute_total_distance(path):
        """compute total sum of distance travelled from path list"""
        path_array = np.diff(np.array(path), axis=0)
        segment_distance = np.sqrt((path_array ** 2).sum(axis=1))
        return np.sum(segment_distance)    
    
    def compute_actual_euclidean(position, goal):
        distance =  (((position[0] - goal[0]) ** 2) + 
                           ((position[1] - goal[1]) ** 2))**(1/2)
                           #((position[2] - goal[2]) ** 2))**(1/2)
        
        return distance

    def compute_quality(best, measured):
        """returns the quality of the solution as twice"""
        quality = abs(measured/best)
        
        return quality
    
    def plot_subplots(num_plots,df_list, column_name,x_label, main_title):
        """plots histogram"""
        n_rows = int(num_plots/2)
        n_cols = int(num_plots/2) 
        c = 1
        fig, axes = plt.subplots(n_rows,n_cols,sharey=False)        
        axes = axes.flat 

        for i, df in enumerate(df_list[0:4]):
            """FREQUENCY -> """
            sns.histplot(df[column_name], bins='auto', color='steelblue', ax=axes[i], 
                         stat='probability', kde=True)
            #sns.kdeplot(data=df[column_name], ax=axes[i],shade=True)
            axes[i].set_title(str((i+1)*10) + " for " + str(len(df)) + " simulations")
            axes[i].set_xlabel(x_label)
            axes[i].set_ylabel("Percent Frequency")
            
        fig.suptitle(main_title)
        
    def get_mission_info(uav_dict, uav_key):
        """returns mission info from pkl file in dictionary"""
        return n_uav_dictionary[uav_key]["Location"]

    def get_quality_solution(mission_index_list, pkl_list):
        """find the quality of the solution path length"""
        solution_list = []
        for i in range(len(mission_index_list)):
            mission_dict = pkl_list[mission_index_list[i]]
            all_waypoints = mission_dict["overall_paths"]
            start_list = mission_dict["start_list"]
            goal_list = mission_dict["goal_list"]
            
            """best waypoints"""
            for i, waypoints in enumerate(all_waypoints):
                if waypoints:
                    twod_coords = [list(wp[:-1]) for wp in waypoints]
                    act_dist = compute_total_distance(twod_coords)
                    best_dist = compute_actual_euclidean(start_list[i], goal_list[i])
                    if compute_quality(best_dist, act_dist) <= 1:
                        solution_list.append(1)
                    elif math.isnan(compute_quality(best_dist, act_dist)) or math.isinf(compute_quality(best_dist, act_dist)):
                        continue
                    else:
                        solution = compute_quality(best_dist, act_dist)
                        solution_list.append(truncate(solution, 3))
                        
                else:
                    solution_list.append(0)
                    
        percent_solution_quality = [i for i in solution_list if i != 0]
        return percent_solution_quality
    
            
    ## open up nuav dictionary 
    ## key in some number of uavs
    uav_keys = list(n_uav_dictionary.keys())
    quality_sol_list = []
    
    ## get list of indexes from index
    for uav in uav_keys[1:]:
        mis_index_list = get_mission_info(n_uav_dictionary, uav)
        qual = get_quality_solution(mis_index_list, pkl_list)
        quality_sol_list.append(qual)


#%% PLOT quality of solutions
    plt.close('all')
    def plot_subplots(num_plots,df_list, column_name,x_label, main_title, transform_type=None):
        """plots histogram"""
        n_rows = int(num_plots/2)
        n_cols = int(2) 
        c = 1
        fig, axes = plt.subplots(n_rows,n_cols,sharey=False)        
        axes = axes.flat 

        for i, df in enumerate(df_list):
            """FREQUENCY -> """
            if transform_type =="log":
                sns.histplot(np.log(df), bins='auto', color='steelblue', ax=axes[i], 
                             stat='probability', kde=True)
            if transform_type == None:
                sns.histplot(df, bins='auto', color='steelblue', ax=axes[i], 
                             stat='probability', kde=True)
                
            if transform_type == "sqrt":
                sns.histplot(np.sqrt(df), bins='auto', color='steelblue', ax=axes[i], 
                             stat='probability', kde=True)
                
            #sns.kdeplot(data=df[column_name], ax=axes[i],shade=True)
            axes[i].set_title(str((i+1)*10) + " for " + str(len(df)) + " UAVS")
            #axes[i].set_xlabel(x_label)
            axes[i].set_ylabel("Count Percent")
        fig.suptitle(main_title)
        fig.tight_layout()
            
        
    plot_subplots(4, quality_sol_list[0:4], "hi", "Something", "Quality of solution")
    # sns.histplot(np.log(quality_sol_list[1]), bins='auto', color='steelblue', 
    #               stat='probability', kde=True)
    
    # sns.histplot(abstract_percent, bins='auto', color='steelblue', 
    #               stat='probability', kde=True)

    # sns.histplot(actual_to_abstract, bins='auto', color='steelblue', 
    #               stat='probability', kde=True)
            
    
    #%% Find mean and standard deviation from quality of solution
    ## does my quality of solutions go down with the number of uavs?
    import statistics    
    plt.close('all')

    def my_mean(x):
        return np.average(x, weights=np.ones_like(x) / x.size)
    def get_mean_std(info):
        """return mean and std from list"""
        #info = info[np.logical_not(np.isnan(info))]                
        #mean = statistics.mean(info)
        #std = statistics.stdev(info)
        info = np.array(info, dtype=float)
        mean = np.nanmean(info)
        std = np.nanstd(info)
        print("mean", mean)
        return mean, std
    
    def plot_mean_std_list(some_list, titlename, xlabel, ylabel):
        """plot mean and std dev"""
        fig = plt.figure()
        sns.set_style("darkgrid")
        fontsize = 16
        colors = sns.color_palette("hls", n_colors=len(some_list))
        
        for i, vals in enumerate(some_list):
            #print(vals)
            mean, std = get_mean_std(vals)
            plt.errorbar(int(i+1)*10, mean, std/2, linestyle='None', marker='o',
                         color=colors[i], capsize=3)
            
        x_range = np.arange(0,(len(some_list)*10)+11, 10)
        plt.xticks(x_range[1:-1])
        plt.xlabel(xlabel, fontsize=fontsize)
        plt.ylabel(ylabel, fontsize=fontsize)
        plt.title(titlename,fontsize=fontsize)
        plt.tight_layout()
        save_image(titlename, fig)
    
    plot_mean_std_list(quality_sol_list[0:4], "Quality of solution", "Number of UAS", "Distance Factor")
    
    
    # test = statistics.mean(quality_sol_list[1])
    # test_std = statistics.stdev(quality_sol_list[1])

#%% Evaluate number of uavs with quality of solution? 
# the first uav should have the best solution so compare first uav and last uav solution path
    
    



    
