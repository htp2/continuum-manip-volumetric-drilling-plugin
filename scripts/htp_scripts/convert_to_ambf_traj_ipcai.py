import numpy as np
import os

traj_dir_name_anat = "/home/henry/exps/20221113_ipcai_directed/trajectories/anatomy"
traj_dir_name_ambf = "/home/henry/exps/20221113_ipcai_directed/trajectories/ambf"
#if traj_dir_name_ambf does not exist, create it
if not os.path.exists(traj_dir_name_ambf):
    os.mkdir(traj_dir_name_ambf)

volume_path = "/home/henry/snake_registration/simulation/anatomy/Seg.nrrd"

# find all files ending in .fcsv
import glob
traj_files = glob.glob(traj_dir_name_anat + "/*.csv")
# remove files with 'params' in the name
traj_files = [traj_file for traj_file in traj_files if 'params' not in traj_file]
#find filename before .csv
for traj_file in traj_files:
    traj_id = traj_file.split('/')[-1].split('.')[0]

    # run a system command xreg-setup_traj_from_3dslicer_traj(< volume path > < 3dslicer traj path > < output path >)
    import os
    os.system(f"xreg-setup_traj_from_3dslicer_traj {volume_path} {traj_dir_name_anat}/{traj_id}.fcsv {traj_dir_name_ambf}/{traj_id}.csv")