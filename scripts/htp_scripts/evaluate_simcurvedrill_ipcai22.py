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


from PyKDL import Frame, Rotation, Vector


@dataclass
class TopicID:
    topic_name: str
    short_name: str

def merge_topics_in_bag(bagreader, topic_ids: Iterable[TopicID] ) -> pd.DataFrame:
    dataframes = []
    for topic in topic_ids:
        df = pd.read_csv(bagreader.message_by_topic(topic.topic_name))
        df.columns = df.columns.map(lambda x: str(x) + topic.short_name if x != "Time" else x)
        dataframes.append(df)
    
    df_cat = dataframes[0]
    for df in dataframes[1:]:
        df_cat = pd.merge_asof(df_cat, df, on="Time", tolerance=0.050)  
    return df_cat


def extract_ambf_RigidBodyState_from_rosbag(df: pd.DataFrame, short_name: str, trans_scale: float):
    return extract_PoseStamped_from_rosbag(df, short_name, trans_scale=trans_scale) # just so happens to have the same naming

def extract_PoseStamped_from_rosbag(df: pd.DataFrame, short_name: str, trans_scale=1.0):
    N = len(df)
    tfs = np.empty((N,1),dtype=Frame)
    rot_str = "pose.orientation."
    tran_str = "pose.position."
    for row in range(N):
        tfs[row,0] = Frame(Rotation.Quaternion(df.loc[row, rot_str+"x"+short_name], 
                                             df.loc[row, rot_str+"y"+short_name],  
                                             df.loc[row, rot_str+"z"+short_name],  
                                             df.loc[row, rot_str+"w"+short_name]), 
                         Vector(df.loc[row, tran_str+"x"+short_name]*trans_scale,
                                df.loc[row, tran_str+"y"+short_name]*trans_scale,
                                df.loc[row, tran_str+"z"+short_name]*trans_scale)) #q=[xyzw]
    return tfs

def find_nearest(df, colname, val):
    return (val-df[colname]).abs().argsort()[:1][0]

def process_single_bag(bag_filename):

    topics = []
    topics.append(TopicID("/ambf/env/snake_stick/State", "cm_base"))
    topics.append(TopicID("/ambf/env/seg27/State", "cm_tip"))
    topics.append(TopicID("/SystemTask/system/measured_cp", "pred_tip"))
    topics = np.array(topics)

    # bag_filename = "/home/henry/exps/20221102_ipcai22_firstpass/20221109171437/s3_e8.0.bag"

    directory = bag_filename[:bag_filename.rfind("/")+1]
    bag_name = bag_filename.split("/")[-1]

    # remove the .bag extension
    bag_prefix = bag_name[:bag_name.rfind(".")]

    # string is of the form "s3_e8.0" and I want to extract the 3 and 8.0
    s = bag_prefix.find("s")
    e = bag_prefix.find("_", s)
    num_slices = int(bag_prefix[s+1:e])
    cam_err = float(bag_prefix[e+2:])
    print(num_slices, cam_err)

    b = bagreader(bag_filename)

    # df = merge_topics_in_bag(b, topics)

    dataframes = []
    for topic in topics:
        print(topic)
        df = pd.read_csv(b.message_by_topic(topic.topic_name))
        df.columns = df.columns.map(lambda x: str(x) + topic.short_name if x != "Time" else x)
        dataframes.append(df)
    
    df_cat = dataframes[0]
    for df in dataframes[1:]:
        df_cat = pd.merge_asof(df_cat, df, on="Time", tolerance=0.050)  
    df=df_cat

    ambf_scale_to_mm = 1.0/10.0

    # w: world, r: robot, 

    w_T_CMbase = extract_ambf_RigidBodyState_from_rosbag(df, "cm_base", trans_scale=ambf_scale_to_mm)
    w_T_CMtip = extract_ambf_RigidBodyState_from_rosbag(df, "cm_tip", trans_scale=ambf_scale_to_mm)
    r_predT_CMtip = extract_PoseStamped_from_rosbag(df, "pred_tip")


    kalman_update_trigger_topicname = "/snake_imager/tip_pos_estimate_imgonly_relative_checked"
    img_index = []
    # check if a kalman_update_trigger_topicname is column name in bagfile
    if (b.topic_table['Topics'] == kalman_update_trigger_topicname).any():
        df2 = pd.read_csv(b.message_by_topic(kalman_update_trigger_topicname))
        for val in df2['Time']:
            img_index.append(find_nearest(df, "Time", val))
    
    if num_slices > len(img_index):
        print(f"Early termination of simulation: num_slices:{num_slices} > len(img_index):{len(img_index)}")
        return

    df3 = pd.read_csv(b.message_by_topic("/SystemTask/holding_at_goal"))

    df_at_target = df3[(df3['data'].ne(df3['data'].shift())==True) & (df3['data'] == True)]
    at_goal_index = []
    for val in df_at_target['Time']:
        at_goal_index.append(find_nearest(df, "Time", val))

    plan = pd.read_csv("/home/henry/bigss/catkin_ws/src/continuum-manip-volumetric-drilling-plugin/resources/axis.csv",names=['x','y','z'])
    plan['x']*=ambf_scale_to_mm
    plan['y']*=ambf_scale_to_mm
    plan['z']*=ambf_scale_to_mm

    actx = np.array([T.p.x() for T in w_T_CMtip[:,0]])
    acty = np.array([T.p.y() for T in w_T_CMtip[:,0]])
    actz = np.array([T.p.z() for T in w_T_CMtip[:,0]])

    predx =np.array([T.p.x() for T in r_predT_CMtip[:,0]])
    predy =np.array([T.p.y() for T in r_predT_CMtip[:,0]])
    predz =np.array([T.p.z() for T in r_predT_CMtip[:,0]])

    fig, ax = plt.subplots()
    fig.subplots_adjust(bottom=0.25)

    axslider = fig.add_axes([0.25, 0.1, 0.65, 0.03])
    slider = Slider(
        ax=axslider,
        label='Location on Trajectory',
        valmin=0.0,
        valmax=1.0,
        valinit=0.5,
    )

    ax.scatter(actx,acty, s=2)
    ax.scatter(predx,predy, s=2)
    ax.scatter(plan['x'],plan['y'], s=2)
    ax.legend(('actual','pred','plan'))
    highlight, = ax.plot(actx[0],acty[0], marker="X", markersize=20, linestyle='')
    highlight2, = ax.plot(predx[0], predy[0], marker="X", markersize=20, linestyle='')


    N = len(actx)-1

    predCMTip_T_CMTip = [T1.Inverse()*T2 for T1,T2 in zip(r_predT_CMtip[:,0], w_T_CMtip[:,0])]
    norm_err = [v.p.Norm() for v in predCMTip_T_CMTip]

    fig2, (ax_x, ax_y, ax_z, ax_err) = plt.subplots(4,1)
    ax_x.plot(actx)
    ax_x.plot(predx)

    ax_y.plot(acty)
    ax_y.plot(predy)

    ax_z.plot(actz)
    ax_z.plot(predz)



    ax_err.plot(norm_err)
    highlight3, = ax_err.plot(0,norm_err[0], marker="X", markersize=20, linestyle='')

    for axis in zip([ax_x, ax_y, ax_z, ax_err]):
        #make vertical line at value
        for i in img_index:
            axis[0].axvline(x=i, color='r', linestyle='--', alpha=0.5)


    ax.plot(actx.take(img_index),acty.take(img_index), marker="X", color='black', markersize=10, linestyle='')

    ax.plot(predx.take(at_goal_index[0::3]),predy.take(at_goal_index[0::3]), marker="X",  markersize=10, linestyle='')
    ax.plot(predx.take(at_goal_index[1::3]),predy.take(at_goal_index[1::3]), marker="X",  markersize=10, linestyle='')
    ax.plot(predx.take(at_goal_index[2::3]),predy.take(at_goal_index[2::3]), marker="X",  markersize=10, linestyle='')
    ax.legend(('actual','pred','plan','act_i','pred_i','img','goal0','goal1','goal2'))

    def update(val):
        i = round(val*N)
        highlight.set_xdata(actx[i])
        highlight.set_ydata(acty[i])

        highlight2.set_xdata(predx[i])
        highlight2.set_ydata(predy[i])

        fig.canvas.draw_idle()

        highlight3.set_xdata(i)
        highlight3.set_ydata(norm_err[i])

        fig2.canvas.draw_idle()

    slider.on_changed(update)


    # stats
    final_goal_index = at_goal_index[-2] # since the last one is the end of the return trip
    final_target = np.array([plan['x'].iloc[-1], plan['y'].iloc[-1], plan['z'].iloc[-1]])
    act_at_final_goal = np.array([actx[final_goal_index], acty[final_goal_index], actz[final_goal_index]])
    final_err = np.linalg.norm(final_target - act_at_final_goal)

    # cumulative sum of act
    distance = np.linalg.norm(np.diff(np.array([actx,acty,actz]).T, axis=0), axis=1)
    cum_distance = distance[~np.isnan(distance)].sum()

    # cumulative sum of plan
    plan_distance = np.linalg.norm(np.diff(np.array([plan['x'],plan['y'],plan['z']]).T, axis=0), axis=1).sum()

    #find the first close point in act to the first point in the plan
    actx_no_na = actx; actx_no_na[np.isnan(actx_no_na)] = 0
    acty_no_na = acty; acty_no_na[np.isnan(acty_no_na)] = 0
    actz_no_na = actz; actz_no_na[np.isnan(actz_no_na)] = 0

    first_close_index = np.argmin(np.linalg.norm(np.array([actx_no_na,acty_no_na,actz_no_na]).T - np.array([plan['x'].iloc[0], plan['y'].iloc[0], plan['z'].iloc[0]]).T, axis=1))

    actx_during_insertion = actx[first_close_index:final_goal_index]
    acty_during_insertion = acty[first_close_index:final_goal_index]
    actz_during_insertion = actz[first_close_index:final_goal_index]

    act_during_insertion = np.array([actx_during_insertion, acty_during_insertion, actz_during_insertion]).T
    plan_array = plan_array = np.array([plan['x'],plan['y'],plan['z']]).T

    # Find the distance of element in act_during insertion to the closest point in plan
    distance_to_plan = np.zeros(len(act_during_insertion))
    for i, x in enumerate(act_during_insertion):
        distance_to_plan[i] = (np.min(np.linalg.norm(plan_array - x, axis=1)))

    #max deviation from plan
    max_deviation = np.max(distance_to_plan)
    #mean deviation from plan
    mean_deviation = np.mean(distance_to_plan)
    #std deviation from plan
    std_deviation = np.std(distance_to_plan)

    predx_no_na = predx; predx_no_na[np.isnan(predx_no_na)] = 0
    predy_no_na = predy; predy_no_na[np.isnan(predy_no_na)] = 0
    predz_no_na = predz; predz_no_na[np.isnan(predz_no_na)] = 0
    predx_during_insertion = predx[first_close_index:final_goal_index]
    predy_during_insertion = predy[first_close_index:final_goal_index]
    predz_during_insertion = predz[first_close_index:final_goal_index]

    pred_during_insertion = np.array([predx_during_insertion, predy_during_insertion, predz_during_insertion]).T

    distance_pred_to_plan = np.zeros(len(pred_during_insertion))
    for i, x in enumerate(pred_during_insertion):
        distance_pred_to_plan[i] = (np.min(np.linalg.norm(plan_array - x, axis=1)))

    #max deviation from plan
    max_deviation_pred = np.max(distance_pred_to_plan)
    #mean deviation from plan
    mean_deviation_pred = np.mean(distance_pred_to_plan)
    #std deviation from plan
    std_deviation_pred = np.std(distance_pred_to_plan)

    fig_err2, ax_err2 = plt.subplots()
    ax_err2.plot(distance_to_plan)
    ax_err2.plot(distance_pred_to_plan)
    ax_err2.plot(norm_err[first_close_index:final_goal_index])
    ax_err2.legend(('act_to_plan','pred_to_plan','act_to_pred'))



    pickle.dump(fig,open(f"{directory}{bag_prefix}/{bag_prefix}_tip_pos.pickle",'wb'))
    pickle.dump(fig2,open(f"{directory}{bag_prefix}/{bag_prefix}_tip_pos_err.pickle",'wb'))
    pickle.dump(fig_err2,open(f"{directory}{bag_prefix}/{bag_prefix}_tip_err_during_insertion.pickle",'wb'))

    # save all stats to csv file
    stats = pd.DataFrame({'final_err': [final_err], 'cum_distance': [cum_distance], 'max_deviation': [max_deviation], 'mean_deviation': [mean_deviation], 'std_deviation': [std_deviation], 'max_deviation_pred': [max_deviation_pred], 'mean_deviation_pred': [mean_deviation_pred], 'std_deviation_pred': [std_deviation_pred], 'num_slices': [num_slices], 'cam_err': [cam_err]})
    stats.to_csv(f"{directory}{bag_prefix}/{bag_prefix}_stats.csv")

    all_stats_filename = f"{directory}combined_stats.csv"
    add_head = not os.path.isfile(all_stats_filename)
    stats.to_csv(all_stats_filename, mode='a',header=add_head)
    plt.close('all')

def process_all_valid(bag_file):
    # try to process bag file
    try:
        process_single_bag(bag_file)
    except:
        print(f"Error processing {bag_file}")
        return
    print(f"Processed {bag_file}")

if __name__ == "__main__":
    
    # Initialize parser
    parser = argparse.ArgumentParser()
    
    # add arguments for directory 
    parser.add_argument("--directory", "-d", type=str, help="directory of bag files")
    # add optional argument for check_if_complete default True
    parser.add_argument("--check_if_complete", "-c", type=bool, default=True, help="check if bag file has already been processed")
    # add optional argument for num_threads default 1
    parser.add_argument("--num_threads", "-n", type=int, default=1, help="number of threads to use")

    args, unknown = parser.parse_known_args()
    directory = args.directory
    check_if_complete = args.check_if_complete
    num_threads = args.num_threads

    print(f"directory: {directory}")
    # get all bag files in directory
    bag_files = glob.glob(f"{directory}*.bag")
    #exclude all that have _all in the name
    bag_files = [x for x in bag_files if '_all' not in x]

    print(bag_files)
    # run process_single_bag on each file in bag_files

    if check_if_complete:
        bag_files_run = []
        for bag_file in bag_files:
            bag_name = bag_file.split("/")[-1]
            bag_prefix = bag_name[:bag_name.rfind(".")] 
            if os.path.isfile(f"{directory}{bag_prefix}/{bag_prefix}_tip_pos.pickle"):
                print(f"{bag_prefix} already processed")
            else:
                bag_files_run.append(bag_file) 

    # parallelize the process
    import multiprocessing
    with multiprocessing.Pool(num_threads) as pool:
        pool.map(process_all_valid, bag_files_run)