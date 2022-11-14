
import subprocess
import os
import datetime
import random
import itertools
import numpy as np
import glob
import time
robot_traj_dir = "/home/henry/exps/20221113_ipcai_directed/trajectories/robot/"
anat_traj_dir = "/home/henry/exps/20221113_ipcai_directed/trajectories/anatomy/"
ambf_traj_dir = "/home/henry/exps/20221113_ipcai_directed/trajectories/ambf/"
simul_traj_dir = "/home/henry/exps/20221113_ipcai_directed/trajectories/simul/"
bash_name = "/home/henry/bigss/catkin_ws/src/continuum-manip-volumetric-drilling-plugin/launch/support_ipcai.sh"

# if simul_traj_dir does not exist, create it
if not os.path.exists(simul_traj_dir):
    os.mkdir(simul_traj_dir)

# find all files ending in csv in the robot trajectories directory
robot_traj_files = glob.glob(robot_traj_dir + "*.csv")
# remove files with 'params' in the name
robot_traj_files = [
    traj_file for traj_file in robot_traj_files if 'params' not in traj_file]
# find filename before .csv
robot_traj_ids = [traj_file.split('/')[-1].split('.')[0]
                  for traj_file in robot_traj_files]
# remove _traj from robot_traj_ids
robot_traj_ids = [traj_id.replace('_traj', '')
                  for traj_id in robot_traj_ids]

# create list from 0 to 10 by 5
cam_err = np.arange(0, 1, 1)
num_slices = np.arange(0, 11, 2)
ids = np.arange(0, len(robot_traj_ids), 1)

# make a list of all combinations of the above
combinations = list(itertools.product(ids, cam_err, num_slices))

# shuffle combinations
random.shuffle(combinations)

# get YmdHMS timestamp
now = datetime.datetime.now()
timestamp = now.strftime("%Y%m%d%H%M%S")
outdir_main = f"{simul_traj_dir}{timestamp}"
os.mkdir(outdir_main)
# run system command
for combination in combinations:
    traj_id = robot_traj_ids[combination[0]]
    cam_err = combination[1]
    num_slices = combination[2]
    outdir = f"{outdir_main}{traj_id}"
    # if the directory does not exist, create it
    if not os.path.exists(outdir):
        os.mkdir(outdir)
    # run a system command support_ipcai.sh and wait for it to finish
    cmd = ['bash', bash_name, cam_err, num_slices,
           traj_id, ambf_traj_dir, robot_traj_dir, outdir]
    # make all in cmd strings
    cmd = [str(x) for x in cmd]
    # cmd = f"bash {bash_name} {cam_err} {num_slices} {traj_id} {ambf_traj_dir} {robot_traj_dir} {outdir}"
    subprocess.Popen(cmd).wait()
    time.sleep(5)
