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
        with open('replay_fails/'+filename, 'wb') as outp:  # Overwrites any existing file.
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

def return_uav_dict(pkl_list, pkl_filenames):
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
        
    return n_uav_dictionary

def save_image(image_name, fig):
    """saves image"""
    image_format = 'svg' # e.g .png, .svg, etc.
    # image_name = 'myimage.svfg'
    
    fig.savefig('images/'+image_name+'.svg', format=image_format, dpi=1200)
    
def return_pkl_dirs(folder_dir_name, index_val):
    """return pkl files from directory"""
    pkl_dirs = []
    for i in range(1,index_val):
        folder_name = folder_dir_name+str(i)
        path = os.getcwd() + "/"+ folder_name
        pkl_file_dirnames = dataparser.get_all_pkl_names(path)
        print(pkl_file_dirnames)
        pkl_dirs.extend(pkl_file_dirnames)
        
    return pkl_dirs

def return_pkl_files(pkl_dirs):
    """return pkl filenames"""
    pkl_list = []
    pkl_filenames = []
    for pkl in pkl_dirs:
        pkl_list.append(dataparser.load_pkl_file(pkl))
        pkl_filenames.append(dataparser.get_filename(pkl))

    return pkl_list, pkl_filenames


def return_df_list(uav_dictionary):
    """return list of dataframes for n uavs"""
    df_list = []
    
    for key,overall_dict in uav_dictionary.items():
        overall_df = pd.DataFrame.from_dict(overall_dict)
        df_list.append(overall_df)

    return df_list

#%% Loading the dictionaries 
if __name__=='__main__':
    
    dataparser = DataParser()
    
    #pkl_dirs = []
    
    #change name for which file directory
    basic_alg_file = "logs/sim_"
    max_prior = "logs/max_prior_0"
    min_prior = "logs/min_prior_0"
    
    max_prior 
    pkl_dirs = return_pkl_dirs(basic_alg_file, 10)
    
    sorted_pkl_dirs = return_pkl_dirs(max_prior, 7)
    
    min_pkl_dirs = return_pkl_dirs(min_prior, 7)    
    # for i in range(1,5):
    #     folder_name = max_prior+str(i)
    #     path = os.getcwd() + "/"+ folder_name
    #     pkl_file_dirnames = dataparser.get_all_pkl_names(path)
        
    #     pkl_dirs.extend(pkl_file_dirnames)
    
#%% 
        
    pkl_list, pkl_filenames = return_pkl_files(pkl_dirs)
    
    sorted_pkl_list, sorted_pkl_filenames = return_pkl_files(sorted_pkl_dirs)
    
    min_pkl_list, min_pkl_filenames = return_pkl_files(min_pkl_dirs)
    
#%% Splitting up the dictionaries based on UAVs, for now..
    DICT_KEYS = ["abstract_paths", 
                 "goal_list", 
                 "iter_list", 
                 "success", 
                 "time_list"]
    
    keys = list(pkl_list[0].keys())
    
    n_uav_dictionary = return_uav_dict(pkl_list, pkl_filenames)
    
    sorted_n_uav_dict = return_uav_dict(sorted_pkl_list, sorted_pkl_filenames)
    
    min_n_uav_dict = return_uav_dict(min_pkl_list, min_pkl_filenames)
    
    

        
#%% Convert overall dictionaries to dataframes for easier formatting and parsing
    df_list = return_df_list(n_uav_dictionary)
    
    sorted_df_list = return_df_list(sorted_n_uav_dict)
    
    min_df_list = return_df_list(min_n_uav_dict)
    
    
    uav_range = np.arange(10,130,10)
    
    for i, df in enumerate(df_list):
        df['N_UAVS']= uav_range[i]
        
    
    for i, df in enumerate(sorted_df_list):
        df['N_UAVS']= uav_range[i]
    
    
    for i, df in enumerate(min_df_list):
        df['N_UAVS']= uav_range[i]
    
    df_range = df_list[0:10]
    sorted_df_range = sorted_df_list[0:10]
    min_df_range = min_df_list[0:10]
    
    overall_df = pd.concat(df_range)
    
    sorted_overall_df = pd.concat(sorted_df_range)
    
    min_overall_df = pd.concat(min_df_range)
    
    
#%% Testing out PLOTS
    
    # plt.close('all')
    # # #https://stackoverflow.com/questions/66446681/python-use-bins-from-one-sns-histplot-for-another-extract-bin-information
    # import seaborn as sns
    
    # from itertools import cycle
    # def plot_subplots(num_plots,df_list, column_name,x_label, main_title,map_list,transform_type=None):
        
    #     #https://stackoverflow.com/questions/66446681/python-use-bins-from-one-sns-histplot-for-another-extract-bin-information
    #     sns.set_style("dark")
    #     colors = sns.color_palette("hls", n_colors=len(df_list))
    #     """plots histogram"""
    #     n_rows = int(3)
    #     n_cols = int(2) 
    #     c = 1
    #     fig, axes = plt.subplots(n_rows,n_cols,sharey=False)        
    #     axes = axes.flat 
    #     for i, df in enumerate(df_list):
    #         """FREQUENCY -> """
    #         bins = np.histogram_bin_edges(df[column_name], bins='scott')
            
    #         if transform_type =="log":
    #             sns.histplot(np.log(df[column_name]), bins=bins, color=colors[i], ax=axes[i], 
    #                           stat='probability', kde=False)
    #         if transform_type == None:
    #             sns.histplot(df[column_name], bins=bins, color=colors[i], ax=axes[i], 
    #                           stat='probability', kde=False)
                
    #         axes[i].set_title(str(map_list[i]) + " for " + str(len(df)) + " simulations")
    #         axes[i].set_ylabel("Count Percent")
    #     fig.suptitle(main_title)    
    #     fig.tight_layout()
    #     save_image(main_title, fig)
        
    
    # n_uavs = np.arange(10,100+10,10)
    # df_range = df_list[0:10]
    # for i,df in enumerate(df_list):
    #     print(i)
    #     if i % 5 == 0:
    #         if i == 0:
    #             continue
    #         plot_subplots(10,df_range[i-5:i], "Success", "Success Rate", 
    #                       "Overall Success for " +str(n_uavs[i-5]) + " to " + str(n_uavs[i-1]),n_uavs[i-5:i])
        
    #         ## why do I have to log transform this guy??    
    #         plot_subplots(10,df_range[i-5:i], "Total Time", "Time Seconds", "Time to Plan Path for " +str(n_uavs[i-5]) + " to " + str(n_uavs[i-1]),
    #                       n_uavs[i-5:i])
            
    #         # whats with the iteration mini peaks?
    #         plot_subplots(10,df_range[i-5:i], "Iter Time", "Number of Iterations", 
    #                       "Total Number of Iterations for " +str(n_uavs[i-5]) + " to " + str(n_uavs[i-1]),n_uavs[i-5:i])
        
    #     else:
    #         continue
        
#%% Ridge plots
    # plot
    plt.close('all')
    import seaborn as sns
    n_uavs = np.arange(10,100+10,10)
    
    def plot_ridge(df, column_name, color_index, title_name,height_offset, txt_space, xlabel):
    
        sns.set_theme(style="white", rc={"axes.facecolor": (0, 0, 0, 0), 'axes.linewidth':2},font_scale=2.0)    #sns.set_theme(style="white")
        #blue -> red 
        #pal = sns.color_palette("icefire")
        #colors = ["icefire", "coolwarm","vlag"]
        colors = ["dark:salmon_r", "BuGn", "Blues_r"]
        
        pal = sns.color_palette(colors[color_index], n_colors=len(n_uavs))
        if color_index == 2:
            pal.reverse()

        g = sns.FacetGrid(df, row="N_UAVS", hue="N_UAVS", aspect=8, height=0.75, palette=pal)
        
        g.map(sns.kdeplot, column_name, fill=True, alpha=1)
        g.map(sns.kdeplot, column_name,color='black')
        
        # g.map(sns.histplot, column_name, fill=True, alpha=1)
        # g.map(sns.histplot, column_name,color='black')
        
        
        g.map(plt.axhline, y=0, lw=2, clip_on=False)

        
        def label(x, color, label):
            ax = plt.gca()
            ax.text(-0.1, txt_space, label, color='black', fontsize=20,
                    ha="left", va="center", transform=ax.transAxes,
                    fontweight='bold')
            
        g.map(label, "N_UAVS")    

        g.refline(y=0, linewidth=2, linestyle="-", color=None, clip_on=False)
        
        g.fig.subplots_adjust(hspace=-height_offset)

        g.set(ylabel=None)
        g.set_titles("")
        g.set(yticks=[], xlabel=xlabel)
        #g.set(ylim=(0, ymax))
        g.set(xlim=(-35, 350))

        g.despine( left=True)
        #plt.suptitle(title_name, y=0.98, fontsize=26)
        save_image(title_name, g)
                

    # plot_ridge(overall_df, "Success", 0, "Unsorted Success Rate Ridge", 0.95, 0.015, "Percent Success")
    # plot_ridge(sorted_overall_df, "Success", 1, "Sorted Success Rate Ridge", .95, 0.015, "Percent Success")
    # plot_ridge(min_overall_df, "Success", 2, "Min Sorted Success Rate Ridge", .95, 0.015, "Percent Success")

    plot_ridge(overall_df, "Total Time", 0, "Unsorted Time Complexity Ridge", 0.925, 0.02, "Time (seconds)")
    # plot_ridge(sorted_overall_df, "Total Time", 1, "Sorted Time Complexity Ridge", .925, 0.02, "Time (seconds)")
    # plot_ridge(min_overall_df, "Total Time", 2, " Min Sorted Time Complexity Ridge", .925, 0.02, "Time (seconds)")
    
    # plot_ridge(overall_df, "Iter Time", 0, "Unsorted Space Complexity Ridge", 0.95, 0.02, "Iterations")
    # plot_ridge(sorted_overall_df, "Iter Time", 1, "Sorted Space Complexity Ridge", .95, 0.02, "Iterations")
    # plot_ridge(min_overall_df, "Iter Time", 2, " Min Sorted Space Complexity Ridge", .95, 0.02, "Iterations")
    
    
#%% Looking at iterations logical indexing of values that are high and checking the distance
    #high_iter_30_uavs = df_list[3].loc[df_list[3]["Iter Time"]>5000]
    #row_1 = high_iter_10_uavs["Index"]
    #situation = pkl_list[276]
    """Evaluate iterations/uav if success"""
    unsorted_failures_dict = {}
    sorted_failures_dict = {}
    min_failures_dict = {}
    
    """
    Why does the path planning fail? Is it an iteration count or something else?
    """
    
    for i, df in enumerate(df_range):
        print("df is", df)
        unsorted_failures_dict [str((i+1)*10)] = df.loc[(df["Success"] != 1)]
        
    for i, df in enumerate(sorted_df_range):
        print("df is", df)
        unsorted_failures_dict [str((i+1)*10)] = df.loc[(df["Success"] != 1)]

    for i, df in enumerate(min_df_range):
        print("df is", df)
        min_failures_dict [str((i+1)*10)] = df.loc[(df["Success"] != 1)]

#%% Identify errors
    from plot_situation import PlotSituation
    import matplotlib.pyplot as plt    
    
    # load_map = True
    # load_graph = True
    # save_information = True
    
    # map_pkl_name = 'map_test.pkl'
    # graph_pkl_name = 'test.pkl'
    

    # if load_map == True:
    #     with open(map_pkl_name, 'rb') as f:
    #         annotated_map  = pickle.load(f)
        
    # if load_graph == True:
    #     with open(graph_pkl_name, 'rb') as f:
    #         graph  = pickle.load(f)
    ####-------- GRAPH 
    def convert_percent_to_actual(total_num,dec_percent):
        """computes the actual value of percentage"""
        return (dec_percent/100)*total_num
    
    """I need to cache this to a database and query it to reduce start up costs
    I should save the information about the ostacles as well or maybe annoted map"""

    #n_uav = 10
    n_range = np.arange(10,10+10,10)
    abstract_fails = []
    #get all dataframes for n uav of the failures
    
    for n_uav in n_range:
        n_uav = 10
        test = min_failures_dict[str(n_uav)]
        fail_pkl_list = []
        fail_pkl_index = []
        
        for idx, row in test.iterrows():
            
            loc_key = row['Location']
            success_rate = row['Success']*100
            fail_dir = min_pkl_dirs[loc_key]
            pkl_info = dataparser.load_pkl_file(fail_dir)
            
            #get number of failures for uavs
            actual = convert_percent_to_actual(total_num=n_uav, dec_percent=success_rate)
            n_fail_uavs = n_uav - actual
            
            #count how many failures I have for uavs based on abstract path and check if it matches n_fails
            abstract_paths = pkl_info['abstract_paths']
            overall_paths = pkl_info['overall_paths']
            
            start_list = pkl_info['start_list']
            goal_list = pkl_info['goal_list']
            
            info = [start_list,goal_list]
            dataparser.save_object(info, 'failure_'+str(n_uav)+str(idx))
            
            #check for how many times I can't find an abtract path
            # for i, abstract in enumerate(abstract_paths):
            #     if isinstance(abstract,int) == True:
            #         cnt=+1
            #         fail_pkl_list.append([abstract,i])
            #     if cnt == int(n_fail_uavs):
            #         #print("Failure is because cant find abstract path")        
            #         #fail_pkl_list.append([abstract,i])
            #         abstract_fails.append(idx)
            #         break
            
            
            
            #abstract_fails.append(len(fail_pkl_list)/len(test))e
        
        
        
    
    
    
    
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
    from matplotlib.lines import Line2D


    def get_mean_std(df, column_name):
        """returns mean and standard deviation from dataframe and column"""
        return df[column_name].mean(), df[column_name].std()
    
    
    def plot_mean_std(df_list,df_list_2,df_list_3, col_name, titlename, xlabel, ylabel, x_range):
        """plot mean and std dev"""
        fig = plt.figure()
        sns.set_style("darkgrid")
        #sns.set_theme(style="whitegrid", rc={"axes.facecolor": (0, 0, 0, 0), 'axes.linewidth':2})    #sns.set_theme(style="white")
        fontsize = 18
        colors= sns.color_palette("Paired", n_colors=len(df_list))
        #colors.reverse()
        custom_lines = [Line2D([0], [0], color=colors[5], lw=4),
                Line2D([0], [0], color=colors[3], lw=4), 
                Line2D([0], [0], color=colors[1], lw=4)]
        
        
        for i, df in enumerate(df_list):
            mean, std = get_mean_std(df, col_name)
            plt.errorbar(int(i+1)*10, mean, std/2, linestyle='-', marker='o',
                         color=colors[5], capsize=1 )
            
        for i, df in enumerate(df_list_2):
            mean, std = get_mean_std(df, col_name)
            plt.errorbar(int(i+1)*10, mean, std/2, linestyle='-.', marker='x',
                         color=colors[3], capsize=1)

        for i, df in enumerate(df_list_3):
            mean, std = get_mean_std(df, col_name)
            plt.errorbar(int(i+1)*10, mean, std/2, linestyle='-.', marker='x',
                         color=colors[1], capsize=1)
            
        #x_range = np.arange(10*offset,50+(offset*10),10)
        plt.legend(custom_lines, ['Unsorted', 'Max Sort', 'Min Sort'])
        plt.xticks(x_range)
        plt.xlabel(xlabel, fontsize=fontsize)
        plt.ylabel(ylabel, fontsize=fontsize)
        #plt.title(titlename,fontsize=fontsize)
        plt.tight_layout()
        
        save_image(titlename, fig)
         
    # test = df_list[0]
    # #Success
    # mean_success, std_success = get_mean_std(test, 'Success')
    # #Iteration time
    # mean_iter, std_iter = get_mean_std(test, 'Iter Time')
    # #Total time
    # mean_time, std_time = get_mean_std(test, 'Total Time')
    
    
    plot_mean_std(df_range, sorted_df_range, min_df_range, "Success", "success_rate", "Number of UAS", "Percent Success",n_uavs)
    plot_mean_std(df_range, sorted_df_range, min_df_range,"Iter Time","space_complexity" ,"Number of UAS", "Number of Iterations",n_uavs)
    plot_mean_std(df_range, sorted_df_range, min_df_range,"Total Time","time_complexity" ,"Number of UAS", "Time (seconds)",n_uavs)
    
    
    #%% PLOT the relationship between
    """
    - Plot success rate vs solution length as scatter plot
    - Plot success rate vs time complexity in scatter plot
    - Compare distance travelled vs optimal distance 
    """
    plt.close("all")
    def plot_success_solution(df_list, df_list_2, col_name, titlename, xlabel,offset):
        """plot mean and std dev"""
        fig = plt.figure()
        sns.set_style("darkgrid")
        fontsize = 16
        colors = sns.color_palette("mako", n_colors=len(df_list))
        colors_2 = sns.color_palette("rocket", n_colors=len(df_list))
        
        colors.reverse()
        colors_2.reverse()
        
        for i, df in enumerate(df_list):
            label = (i+offset)*10
            success = df["Success"]
            time = df[col_name]
            plt.scatter(time, success,c=colors[i], label=label)
            
        for i, df in enumerate(df_list_2):
            label = (i+offset)*10
            success = df["Success"]
            time = df[col_name]
            plt.scatter(time, success,c=colors_2[i], label=label)
            
        plt.xlabel(xlabel, fontsize=fontsize)
        plt.ylabel("Success Rate", fontsize=fontsize)
        plt.legend()
        plt.title(titlename,fontsize=fontsize)
        plt.tight_layout()
        save_image(titlename, fig)
        
    def compare(df_list, df_list2, df_list3, another_col_name,col_name, titlename, xlabel,offset):
        """plot mean and std dev"""
        fig = plt.figure()
        sns.set_style("darkgrid")
        fontsize = 16
        colors = sns.color_palette("rocket", n_colors=len(df_list))
        colors_2 = sns.color_palette("mako", n_colors=len(df_list))
        
        colors.reverse()
        colors_2.reverse()
        
        for i, df in enumerate(df_list):
            label = (i+offset)*10
            plt.scatter(df[col_name], df[another_col_name],c=colors[i], label=label)
            
        for i, df in enumerate(df_list2):
            label = (i+offset)*10
            plt.scatter(df[col_name], df[another_col_name],c=colors_2[i], label=label)
            
        plt.xlabel(xlabel, fontsize=fontsize)
        plt.ylabel(str(another_col_name), fontsize=fontsize)
        plt.legend()
        plt.title(titlename,fontsize=fontsize)
        plt.tight_layout()
        save_image(titlename, fig)
        
    plot_success_solution(df_range, sorted_df_range, "Iter Time", "Iterations vs Success", "Iterations",1)
    plot_success_solution(df_range, sorted_df_range,"Total Time", "Total Time vs Success", "Time (secs)",1)
    #compare(df_range, sorted_df_range,  "Total Time", "Iter Time", "Iterations vs Time", "Iterations",1)
    
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
    
    def compute_quality_percent(best,measured):
        percent_quality = abs(best/measured)
        
        return percent_quality
    
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
        return uav_dict[uav_key]["Location"]

    def get_quality_solution(mission_index_list, pkl_list):
        """find the quality of the solution path length"""
        solution_list = []
        percent_solution = []
        for i in range(len(mission_index_list)):
            mission_dict = pkl_list[mission_index_list[i]]
            all_waypoints = mission_dict["overall_paths"]
            if "start_list" in mission_dict:
                start_list = mission_dict["start_list"]
                goal_list = mission_dict["goal_list"]
            else:
                continue
            
            """best waypoints"""
            for i, waypoints in enumerate(all_waypoints):
                if waypoints:
                    twod_coords = [list(wp[:-1]) for wp in waypoints]
                    act_dist = compute_total_distance(twod_coords)
                    best_dist = compute_actual_euclidean(start_list[i], goal_list[i])
                    if compute_quality(best_dist, act_dist) <= 1:
                        solution_list.append(1)
                        percent_solution.append(1)
                    elif math.isnan(compute_quality(best_dist, act_dist)) or math.isinf(compute_quality(best_dist, act_dist)):
                        continue
                    else:
                        solution = compute_quality(best_dist, act_dist)
                        solution_list.append(truncate(solution, 3))
                        
                        percent_sol = compute_quality_percent(best_dist, act_dist)
                        percent_solution.append(percent_sol)
                        
                else:
                    solution_list.append(0)
                    
        quality_sol= [i for i in solution_list if i != 0]
        percent_qual = [i for i in percent_solution if i != 0]
        return quality_sol, percent_qual
    
            
    ## open up nuav dictionary 
    ## key in some number of uavs
    uav_keys = list(n_uav_dictionary.keys())
    quality_sol_list = []
    percent_qual_list = []
    
    ## get list of indexes from index
    for uav in uav_keys[1:]:
        mis_index_list = get_mission_info(n_uav_dictionary, uav)
        qual, percent_qual = get_quality_solution(mis_index_list, pkl_list)
        quality_sol_list.append(qual)
        percent_qual_list.append(percent_qual)

    sorted_qual_sol_list = []
    sorted_percent_qual = []        
    for uav in uav_keys[1:]:
        mis_index_list = get_mission_info(sorted_n_uav_dict, uav)
        qual, percent_qual = get_quality_solution(mis_index_list, sorted_pkl_list)
        sorted_qual_sol_list.append(qual)
        sorted_percent_qual.append(percent_qual)
        

    min_qual_sol_list = []
    min_percent_qual = []        
    for uav in uav_keys[1:]:
        mis_index_list = get_mission_info(min_n_uav_dict, uav)
        qual, percent_qual = get_quality_solution(mis_index_list, min_pkl_list)
        min_qual_sol_list.append(qual)
        min_percent_qual.append(percent_qual)

#%% PLOT quality of solutions
    # plt.close('all')
    # def plot_subplots(num_plots,df_list, column_name,x_label, main_title,offset, transform_type=None):
    #     """plots histogram"""
    #     n_rows = int(num_plots/2)
    #     n_cols = int(2) 
    #     c = 1
    #     fig, axes = plt.subplots(n_rows,n_cols,sharey=False)        
    #     axes = axes.flat 

    #     for i, df in enumerate(df_list):
    #         """FREQUENCY -> """
    #         if transform_type =="log":
    #             sns.histplot(np.log(df), bins='auto', color='steelblue', ax=axes[i], 
    #                          stat='probability', kde=True)
    #         if transform_type == None:
    #             sns.histplot(df, bins='auto', color='steelblue', ax=axes[i], 
    #                          stat='probability', kde=True)
                
    #         if transform_type == "sqrt":
    #             sns.histplot(np.sqrt(df), bins='auto', color='steelblue', ax=axes[i], 
    #                          stat='probability', kde=True)
                
    #         #sns.kdeplot(data=df[column_name], ax=axes[i],shade=True)
    #         axes[i].set_title(str((i+offset)*10) + " for " + str(len(df)) + " UAVS")
    #         #axes[i].set_xlabel(x_label)
    #         axes[i].set_ylabel("Count Percent")
    #     fig.suptitle(main_title)
    #     fig.tight_layout()
            
    
    # #plot_subplots(4, quality_sol_list[3:7], "hi", "Something", "Quality of solution", 4)
    
    # #quality_sol_range = quality_sol_list[0:10]
    # for i,df in enumerate(quality_sol_range):
    #     print(i)
    #     if i % 5 == 0:
    #         if i == 0:
    #             continue
    #         plot_subplots(10,df_range[i-5:i], "Solution Quality", "Number of UAS", "Quality Solution",n_uavs[i-5:i])
        
    #         ## why do I have to log transform this guy??    
    #         #plot_subplots(10,df_range[i-5:i], "Total Time", "Time Seconds", "Time to Plan Path",n_uavs[i-5:i])
            
    #         # whats with the iteration mini peaks?
    #         #plot_subplots(10,df_range[i-5:i], "Iter Time", "Number of Iterations", "Total Number of Iterations",n_uavs[i-5:i])
        
    #     else:
    #         continue
    
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
    
    def plot_mean_std_list(some_list, titlename, xlabel, ylabel,offset, n_uavs):
        """plot mean and std dev"""
        fig = plt.figure()
        sns.set_style("darkgrid")
        fontsize = 16
        colors= sns.color_palette("mako", n_colors=len(df_list))
        #colors.reverse()
        custom_lines = [Line2D([0], [0], color=colors[4], lw=4),
                Line2D([0], [0], color=colors[8], lw=4)]
        
        mean_list = []
        std_list = []
        for i, vals in enumerate(some_list):
            #print(vals)
            mean, std = get_mean_std(vals)
            mean_list.append(mean)
            std_list.append(std)
            plt.errorbar(int(i+offset)*10, mean, std/2, linestyle='None', marker='o',
                         color=colors[i], capsize=3)
            
        # for i, vals in enumerate(some_list2):
        #     #print(vals)
        #     mean, std = get_mean_std(vals)
        #     plt.errorbar(int(i+offset)*10, mean, std/2, linestyle='None', marker='o',
        #                  color=colors[i], capsize=3)
            
        x_range = n_uavs
        #x_range = np.arange(offset*10,(len(some_list)*(10*offset)), 10)
        print("x_range", x_range)
        plt.xticks(x_range[0:-1])
        plt.xlabel(xlabel, fontsize=fontsize)
        plt.ylabel(ylabel, fontsize=fontsize)
        plt.title(titlename,fontsize=fontsize)
        plt.tight_layout()
        save_image(titlename, fig)
        
        return mean_list, std_list
    
    
    #plot_mean_std_list(quality_sol_list, "Quality of Solution Unsorted", "Number of UAS", "Distance Factor",1, n_uavs)
    unsort_mean, unsort_std = plot_mean_std_list(percent_qual_list, "Percent Quality Unsorted", "Number of UAS", "Distance Factor",1, n_uavs)
    
    #plot_mean_std_list(sorted_qual_sol_list, "Quality of Solution Sorted", "Number of UAS", "Distance Factor",1, n_uavs)
    max_mean, max_std = plot_mean_std_list(sorted_percent_qual, "Percent Quality Sorted", "Number of UAS", "Distance Factor",1, n_uavs)

    min_mean, min_std = plot_mean_std_list(min_percent_qual, "Percent Quality Min Sorted", "Number of UAS", "Distance Factor",1, n_uavs)
    #plot_mean_std_list(min_qual_sol_list, "Quality of Solution Min Sorted", "Number of UAS", "Distance Factor",1, n_uavs)
    
    #as a percent
    #percent_quality = [val-1.0 for val in quality_sol_range]
    
    # test = statistics.mean(quality_sol_list[1])
    # test_std = statistics.stdev(quality_sol_list[1])

#%% Evaluate number of uavs with quality of solution? 
# the first uav should have the best solution so compare first uav and last uav solution path
    
    



    
