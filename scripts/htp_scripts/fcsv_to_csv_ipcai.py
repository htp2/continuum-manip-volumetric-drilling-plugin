import numpy as np
# load file by line
filename = "/home/henry/exps/20221113_ipcai_planned_trajs/traj_entry4_target4.fcsv"

directory = "/home/henry/exps/20221113_ipcai_planned_trajs/"

# find all files ending in .fcsv
import glob
fcsv_files = glob.glob(directory + "*.fcsv")

# for each file, load the points and save them as a csv file
for fcsv_file in fcsv_files:
    with open(fcsv_file, 'r') as f:
        lines = f.readlines()
        # get header
        header = lines[2]
        # get data
        data = lines[3:]
        # get number of columns
        num_cols = len(header.split(','))
        # get number of rows
        num_rows = len(data)
        # create empty array
        arr = np.zeros((num_rows, 3))
        # fill array
        for i, line in enumerate(data):
            arr[i, :] = np.array(line.split(','))[1:4].astype(np.float)
        print(arr)

    # write to csv
    np.savetxt(fcsv_file.replace('.fcsv', '.csv'), arr, delimiter=',')

# write filenames to csv
np.savetxt(directory + "filenames.csv", np.array(fcsv_files), delimiter=',', fmt='%s')
