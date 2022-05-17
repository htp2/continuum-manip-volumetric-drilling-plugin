import bagpy
from bagpy import bagreader
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib.widgets import SpanSelector
import rosbag

class IndexRangeList:
    def __init__(self, figure):
        self.range_list = []
        self.fig = figure

    def onselect(self, xmin, xmax):
        self.range_list.append((xmin,xmax))
        self.fig.fill_betweenx( self.fig.axes.get_ylim(), xmin, xmax)
def scratch():
    b = bagreader("/home/henry/snake_registration/simulation/rosbags/testing1_2022-05-17-11-12-24.bag")
    print(b.topic_table)

    bend_cable = b.message_by_topic('/ambf/volumetric_drilling/bend_motor/measured_js')
    snake_base = b.message_by_topic('/ambf/env/snake_stick/State')
    snake_tip = b.message_by_topic('/ambf/env/seg27/State')
    carm = b.message_by_topic('/ambf/env/carm/State')

    df_bend_cable = pd.read_csv(bend_cable)

    df_snake_base = pd.read_csv(snake_base)
    df_snake_tip = pd.read_csv(snake_tip)
    df_carm = pd.read_csv(carm)

    df_snake_base.columns = df_snake_base.columns.map(lambda x: str(x) + '_snake_base' if x != "Time" else x)
    df_snake_tip.columns = df_snake_tip.columns.map(lambda x: str(x) + '_snake_tip' if x != "Time" else x)
    df_carm.columns = df_carm.columns.map(lambda x: str(x) + '_carm' if x != "Time" else x)

    df_cat = pd.merge_asof(df_snake_base, df_snake_tip, on="Time", tolerance=0.050)
    df_cat2 = pd.merge_asof(df_cat, df_carm, on="Time", tolerance=0.050)


# # here tolerance in seconds
# df_cat = pd.merge_asof(df_robot, df_tool, on="Time", tolerance=0.050)
# df_cat2 = pd.merge_asof(df_cat, df_base, on="Time", tolerance=0.050)

# fig = df_cat2[["transform.translation.z_robot","transform.translation.z_tool","transform.translation.z_base"]].plot()

# df_cat2["Time"] = pd.to_datetime(df_cat2["Time"],unit='s')
# df_cat2 = df_cat2.set_index(["Time"])
# df_cat2[["transform.translation.z_robot","transform.translation.z_tool","transform.translation.z_base"]].resample('0.01S').mean().plot()

# range_list = IndexRangeList(fig)

# span = SpanSelector(fig, range_list.onselect, 'horizontal', useblit=True,
#                     rectprops=dict(alpha=0.5, facecolor='tab:blue'))
# # Set useblit=True on most backends for enhanced performance.


# plt.show()



# print(range_list.range_list)

import argparse
import time
import rosbag

def main():

    parser = argparse.ArgumentParser(description='Reorder a bagfile based on header timestamps.')
    parser.add_argument('bagfile', nargs=1, help='input bag file')
    args = parser.parse_args()
    bagfile = args.bagfile[0]
    print(bagfile)
    bag = rosbag.Bag(bagfile)
    dict = {}
    # with rosbag.Bag('output.bag', 'w') as outbag:
    i=0
    for topic, msg, t in bag.read_messages(topics=['/ambf/env/snake_stick/State']):
        # print(msg)
        if(i>0):
            print(t-t_old)
        t_old = t
        # dict[msg.wall_time] = (topic,msg,t )
        user_in = input()
        i+=1

if (__name__ == "__main__"):
    main()
