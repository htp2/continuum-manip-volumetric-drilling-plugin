import numpy as np
from stl import mesh # pip install numpy-stl
import math

directory = "/home/henry/bigss/catkin_ws/src/continuum-manip-volumetric-drilling-plugin/ADF/meshes/bigss_snake_m_scale/high_res"
# loop through all the files in direct and output a volume estimate

# stifness values in n-m/rad
stiffness_values = [0.784098994, 0.76095515, 0.737811307, 0.714667463, 0.691523619, 0.668379776, 0.645235932, 0.622092089, 0.598948245, 0.575804401, 0.552660558, 0.529516714, 0.50637287, 0.483229027, 0.460085183, 0.436941339, 0.413797496, 0.390653652, 0.367509808, 0.344365965, 0.321222121, 0.305776968, 0.305776968, 0.305776968, 0.305776968, 0.305776968, 0.305776968]


# find all filenames ending with .stl in directory
import os
filenames = []
for file in os.listdir(directory):
    if file.endswith(".stl") or file.endswith(".STL"):
        filenames.append(file)

# nitinol density
d_gcm = 6.45# g/cu.cm
d_kgm = d_gcm * 1000 # kg/cu.m

# create a map from name (with .stl or .STL removed) to mass
mass_map = {}

length_scale = 10.0

# loop through all the files in direct and output a volume estimate
for name in filenames:
    # load the mesh
    mymesh = mesh.Mesh.from_file(directory + "/" + name)
    # calculate the volume
    volume, cog, inertia = mymesh.get_mass_properties()
    # convert to kg
    mass = volume * d_kgm
    # print the volume
    # print(name + " volume: " + str(volume))
    print(name + " mass: " + str(mass))
    print(name + " volume: " + str(volume))
    # add to map
    mass_map[name[:-4]] = float(mass)


import yaml

if True:

    # Load the yaml file
    filename = "/home/henry/bigss/catkin_ws/src/continuum-manip-volumetric-drilling-plugin/ADF/bigss_snake_m_scale.yaml"
    with open(filename, "r") as file:
        data = yaml.load(file, Loader=yaml.FullLoader)

    # Iterate through the data and find bodies
    for key, value in data.items():
        if key.startswith("BODY "):    
            # extract the body name (written as BODY body_name)
            body_name = key[5:]
            # if the body name is in the mass map then find the mass key and update the value
            if body_name in mass_map:
                for subkey, subvalue in value.items():
                    if subkey == "mass":
                        value[subkey] = mass_map[body_name]*math.pow(length_scale,2)


    # Iterate through the data and find occurrences of 'joint1' to 'joint26'
    for key, value in data.items():
        if key.startswith("JOINT joint") and key[11:].isdigit():        
            joint_num = int(key[11:]) # Extract the joint number from the key
            if joint_num in range(1, 27):
                # Find the 'stiffness' key and update the value
                for subkey, subvalue in value.items():
                    if subkey == "stiffness":
                        value[subkey] = stiffness_values[joint_num-1] * math.pow(length_scale, 2)

    # Save the updated data to the yaml file
    # put "_changed" before yaml
    new_filename = filename.replace(".yaml", "_changed.yaml")

    with open(new_filename, "w") as file:
        yaml.dump(data, file)








