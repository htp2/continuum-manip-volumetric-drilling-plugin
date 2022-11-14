from dataclasses import dataclass
from typing import Iterable
from bagpy import bagreader
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import pickle
import os
import argparse
import glob 
from matplotlib.widgets import Slider

if __name__ == "__main__":
    
    # Initialize parser
    parser = argparse.ArgumentParser()
    
    # add arguments for directory 
    parser.add_argument("--directory", "-d", type=str, help="directory of bag files")
    # add optional argument for check_if_complete default True
    parser.add_argument("--check_if_complete", "-c", type=bool, default=True, help="check if bag file has already been processed")

    args, unknown = parser.parse_known_args()
    directory = args.directory
    check_if_complete = args.check_if_complete

    print(f"directory: {directory}")
    # get all bag files in directory

    all_stats_filename = f"{directory}combined_stats.csv"
    
    # read pd from csv
    if os.path.isfile(all_stats_filename):
        all_stats = pd.read_csv(all_stats_filename)
    else:
        print("No stats file found")
        quit()
    
    # plot all stats num_slices vs final_err
    plt.figure()
    plt.scatter(all_stats["num_slices"], all_stats["final_err"])
    plt.xlabel("num_slices")
    plt.ylabel("final_err")
    plt.title("num_slices vs final_err")

    # plot all stats num_slices vs max_deviation
    plt.figure()
    plt.scatter(all_stats["num_slices"], all_stats["max_deviation"])
    plt.xlabel("num_slices")
    plt.ylabel("max_deviation")
    plt.title("num_slices vs max_deviation")

    # plot all stats num_slices vs mean_deviation
    plt.figure()
    plt.scatter(all_stats["num_slices"], all_stats["mean_deviation"])
    plt.xlabel("num_slices")
    plt.ylabel("mean_deviation")
    plt.title("num_slices vs mean_deviation")

    # plot all stats cam_err vs final_err
    plt.figure()
    plt.scatter(all_stats["cam_err"], all_stats["final_err"])
    plt.xlabel("cam_err")
    plt.ylabel("final_err")
    plt.title("cam_err vs final_err")

    # plot all stats cam_err vs max_deviation
    plt.figure()
    plt.scatter(all_stats["cam_err"], all_stats["max_deviation"])
    plt.xlabel("cam_err")
    plt.ylabel("max_deviation")
    plt.title("cam_err vs max_deviation")

    # plot all stats cam_err vs mean_deviation
    plt.figure()
    plt.scatter(all_stats["cam_err"], all_stats["mean_deviation"])
    plt.xlabel("cam_err")
    plt.ylabel("mean_deviation")
    plt.title("cam_err vs mean_deviation")

    # plt.show()

    from statsmodels.formula.api import ols, mixedlm
    lm = ols("mean_deviation ~ cam_err + num_slices + cam_err:num_slices", data=all_stats).fit()
    print(lm.summary2())

    lm = ols("mean_deviation ~ cam_err + num_slices", data=all_stats).fit()
    print(lm.summary2())

    lm = ols("mean_deviation ~ num_slices", data=all_stats).fit()
    print(lm.summary2())

    import scipy.stats as stats
    labels = ["Statistic", "p-value"]

    norm_res = stats.shapiro(lm.resid)

    for key, val in dict(zip(labels, norm_res)).items():
        print(key, val)

    fig = plt.figure(figsize = (16, 9))
    ax = fig.add_subplot(111)
    import statsmodels.api as sm

    sm.qqplot(lm.resid, dist = stats.norm, line = 's', ax = ax)

    ax.set_title("Q-Q Plot")
    plt.show()
    # from statsmodels.stats.diagnostic import het_white

    # het_white_res = het_white(lm.resid, lm.model.exog)

    # labels = ["LM Statistic", "LM-Test p-value", "F-Statistic", "F-Test p-value"]

    # for key, val in dict(zip(labels, het_white_res)).items():
    #     print(key, val)

    # # mixlm = mixedlm("mean_deviation ~ cam_err + num_slices + cam_err:num_slices", data=all_stats).fit()
    # # print(mixlm.summary2())




