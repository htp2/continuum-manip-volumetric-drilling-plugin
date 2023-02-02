#!/usr/bin/env python
# //==============================================================================
# /*
#     Software License Agreement (BSD License)
#     Copyright (c) 2021

#     All rights reserved.

#     Redistribution and use in source and binary forms, with or without
#     modification, are permitted provided that the following conditions
#     are met:

#     * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.

#     * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.

#     * Neither the name of authors nor the names of its contributors may
#     be used to endorse or promote products derived from this software
#     without specific prior written permission.

#     THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#     "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#     LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#     FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#     COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#     INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#     BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#     LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#     CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#     LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#     ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#     POSSIBILITY OF SUCH DAMAGE.

#     \author    <amunawar@jhu.edu>
#     \author    Adnan Munawar

#     \author    <henry.phalen@jhu.edu>
#     \author    Henry Phalen

#     \version   1.0



# */
# //==============================================================================
from numpy import imag
import nrrd
import PIL.Image
import numpy as np
from argparse import ArgumentParser
import sys
from os.path import isdir, isfile
from os import mkdir


def convert_png_transparent(image, bg_color=(255,255,255)):
    # https://stackoverflow.com/questions/765736/how-to-use-pil-to-make-all-white-pixels-transparent
    # Jonathan Dauwe
    array = np.array(image, dtype=np.ubyte)
    mask = (array[:,:,:3] == bg_color).all(axis=2)
    alpha = np.where(mask, 0, 255)
    array[:,:,-1] = alpha
    return PIL.Image.fromarray(np.ubyte(array))
    
def save_image(array, im_name):
    img = PIL.Image.fromarray(array.astype(np.uint8))
    img = img.convert("RGBA")
    img = convert_png_transparent(img, bg_color=(0,0,0))
    img.save(im_name)


def normalize_data(data):
    max = data.max()
    min = data.min()
    if max==min:
        if min!= 0: # assume entire image is single volume
            normalized_data = data/min
        # here, else is implicit - image is all zero and will remain that way
    else:
        normalized_data = (data - min) / float(max - min)
    return normalized_data


def scale_data(data, scale):
    scaled_data = data * scale
    return scaled_data


def save_volume_as_images(data, im_prefix):
    for i in range(data.shape[2]):
        im_name = im_prefix + '0' + str(i) + '.png'
        save_image(data[:, :, i], im_name)

def save_yaml_file(data_size, dimensions, volume_name, yaml_save_location, origin, scale):
    # Note a swap in the 'x' and 'y' coordinates as a result of nrrd being HxW vs WxH for AMBF
    lines = []
    lines.append("# AMBF Version: (0.1)")
    lines.append("bodies: []")
    lines.append("joints: []")
    lines.append("volumes: [VOLUME "+volume_name+"]")
    lines.append("high resolution path: ./meshes/high_res/")
    lines.append("low resolution path: ./meshes/low_res/")
    lines.append("ignore inter-collision: true")
    lines.append("namespace: /ambf/env/")
    lines.append("")
    lines.append("VOLUME "+volume_name+":")
    lines.append("  name: "+volume_name)
    lines.append("  location:")
    lines.append("    position: {x: " + str(origin[1])+", y: "+str(origin[0])+", z: "+str(origin[2])+"}")
    lines.append("    orientation: {r: 0.0, p: 0.0, y: " + str(np.pi/2)+"}")
    lines.append("  scale: "+str(scale))
    lines.append("  dimensions: {x: "+str(dimensions[1])+", y: "+str(dimensions[0])+", z: " + str(dimensions[2]) +"}")
    lines.append("  images:")
    lines.append("    path: ../resources/volumes/"+volume_name+"/")
    lines.append("    prefix: plane00")
    lines.append("    format: png")
    lines.append("    count: " + str(max(data_size)))  # Note this can be larger than actual value
    lines.append("  shaders:")
    lines.append("    path: ./shaders/volume/")
    lines.append("    vertex: shader.vs")
    lines.append("    fragment: shader.fs")
    with open(yaml_save_location+volume_name+".yaml", 'w') as f:
        f.write('\n'.join(lines))
        f.close()


def main():
    # Begin Argument Parser Code
    parser = ArgumentParser()
    parser.add_argument('-n', action='store', dest='nrrd_file', help='Specify Nrrd File')
    parser.add_argument('-p', action='store', dest='image_prefix', help='Specify Image Prefix',
                        default='plane0')
    parser.add_argument('-v', action='store', dest='volume_name', help='Specify the name for volume')
    parser.add_argument('-y', action='store', dest='yaml_save_location', help='Specify path for new yaml file')
    parser.add_argument('-i', action='store', dest='png_img_save_location', help='Specify path for png file directory')

    parsed_args = parser.parse_args()
    print('Specified Arguments')
    print(parsed_args)

    if not isfile(parsed_args.nrrd_file):
        sys.exit("Error: nrrd file " + parsed_args.nrrd_file + " does not exist")

    if not isdir(parsed_args.yaml_save_location):
        user_in = input("Yaml save location " + parsed_args.yaml_save_location + " does not exist, would you like to create it? (Y/n): ")
        if user_in.lower() == "n":
            sys.exit("Exiting... ")
        mkdir(parsed_args.yaml_save_location)
    
    png_img_dir = parsed_args.png_img_save_location+"/"+parsed_args.volume_name+"/"
    if not isdir(png_img_dir):
        user_in = input("PNG Image save location " + png_img_dir + " does not exist, would you like to create it? (Y/n): ")
        if user_in.lower() == "n":
            sys.exit("Exiting... ")
        print("creating :" + png_img_dir)
        mkdir(png_img_dir)	

    scale = 10.0

    data, header = nrrd.read(parsed_args.nrrd_file)
    print(np.min(data))
    if header['space'] != 'left-posterior-superior':
        print("WARNING: Coord system is not LPS, but: "+ header['space'])
    dimensions_mm = np.matmul(header['space directions'],header['sizes'])
    dimensions_m = 0.001*(dimensions_mm)
    origin_mm = scale*(header['space origin'] + (dimensions_mm/2))
    origin_m = 0.001 * origin_mm
    print(header)
    data_size = data.shape
    # scaling = data.shape/np.linalg.norm(data.shape)
    # scaling = data.shape/np.max(data.shape)

    # print("Data size: " +  str(data_size))
    # print("Scaling: " + str(scaling))
    normalized_data = normalize_data(data)
    scaled_data = scale_data(normalized_data, 255.9)
    
    save_volume_as_images(scaled_data, png_img_dir+parsed_args.image_prefix)
    save_yaml_file(data_size, dimensions_m, parsed_args.volume_name, parsed_args.yaml_save_location, origin_m, scale)



if __name__ == '__main__':
    main()
