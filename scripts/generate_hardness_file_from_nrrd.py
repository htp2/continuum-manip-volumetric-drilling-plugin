#!/usr/bin/env python3

import nrrd
import PIL.Image
import numpy as np
from argparse import ArgumentParser
import sys
from os.path import isdir, isfile
from os import mkdir


def main():
    # Begin Argument Parser Code
    parser = ArgumentParser()
    parser.add_argument('-n', action='store', dest='nrrd_file', help='Specify Nrrd File')
    parser.add_argument('-o', action='store', dest='output_dir', help='Specify output file')
    parsed_args = parser.parse_args()
    nrrd_file = parsed_args.nrrd_file
    if not isfile(parsed_args.nrrd_file):
        sys.exit("Error: nrrd file " + parsed_args.nrrd_file + " does not exist")
    scale = 10.0
    output_dir = parsed_args.output_dir
    # check if save location exists
    if not isdir(output_dir):
        # check to see if user wants to create the directory, with a Y/N prompt, if N exit
        print("Output directory does not exist, do you want to create it? (Y/n): ")
        user_input = input()
        if user_input.lower() != 'n':
            mkdir(output_dir)
        else:
            sys.exit("Error: output directory does not exist")

    # get the file name of nrrd file
    nrrd_file_name = nrrd_file.split("/")[-1]
    output_filename = output_dir + "/" + nrrd_file_name.replace(".nrrd", "_hardness.csv")
    
    # read the nrrd file    
    data, header = nrrd.read(nrrd_file)
    if header['space'] != 'left-posterior-superior':
        print("WARNING: Coord system is not LPS, but: " + header['space'])
    dimensions_mm = np.matmul(header['space directions'], header['sizes'])
    dimensions_m = 0.001*(dimensions_mm)
    origin_mm = scale*(header['space origin'] + (dimensions_mm/2))
    origin_m = 0.001 * origin_mm
    print(header)
    
    print("help")

    min_hu = np.min(data[data > -1000])
    max_hu = np.max(data)
    
    # set data[data > -1000] to be scaled between 0 and 1
    if min_hu == max_hu:
        if min_hu == -1000:
            # set all data to 0
            data = np.zeros(data.shape)
        else:
            # set all data to 1
            data = np.ones(data.shape)
    else:
        data[data > -1000] = (data[data > -1000] - min_hu) / (max_hu - min_hu)
        # set data[data <= -1000] to be 0
        data[data <= -1000] = 0

    # data is (x by y by z) but we want (y by x by z)
    data = np.swapaxes(data, 0, 1)
    # we want to flip the y' axis if axes are (x' by y' by z')
    data = np.flip(data, 1)
    data_size = data.shape
    # now let's save to csv where first line is the dimensions of the data (x by y by z) and then nested for loop through x,y,z each line is data[x,y,z]
    with open(output_filename, 'w') as f:
        
        # write the dimensions
        f.write(str(data_size[0]) + "," + str(data_size[1]) + "," + str(data_size[2]) + "\n")
        for x in range(data_size[0]):
            for y in range(data_size[1]):
                for z in range(data_size[2]):
                    f.write(str(data[x,y,z])+ "\n")

    print(f"Hardness file saved to: {output_filename}")


    


if __name__ == '__main__':
    main()

