**This work will be formally introduced as a Long Abstract at IPCAI in June 2023**

# Description

A program to simulate a continuum manipulator that is able to interact with and remove parts of a volumetric model. The initial application is to simulate the control of a dexterous surgical tool for curved drilling for autonomous surgical procedures in orthopaedics (e.g., femur and spine). This is implemented as a plugin for AMBF https://github.com/WPI-AIM/ambf/.
The primary use-case of this simulation tool is to more-rapidly develop and test control strategies and allow for visualization of feasability, etc. of certain plans. When paired with the XREG library, simulated Xray images can be taken of the scene and can be used to train, test, etc. image-based navigation.


https://user-images.githubusercontent.com/17507145/216717871-ee82b41d-9e78-4d6e-a40e-4dd5104f0a77.mp4


https://user-images.githubusercontent.com/17507145/216717892-d7fd4dc4-dcd4-4406-8a1b-ea22bb869bd1.mp4


https://user-images.githubusercontent.com/17507145/232047575-106d8e54-e95c-400b-a68e-ca77c69933a6.mp4

This plugin is an 'unofficial fork' of Adnan Munawar et al. See their work at https://github.com/LCSR-SICKKIDS/volumetric_drilling. Both have undergone significant development since the split so there is some divergence. The plan is to converge at least the volumetric drilling (i.e. what happens at the burr) at some point. In fact, this plugin may be split into several plugins in the future, but for now, it is all in one.

This pairs well with another plugin I wrote (https://github.com/htp2/ambf_trace_plugin), you might find reference to it in the launch file!

# Installation Instructions:
## Install and Source AMBF 2.0

Build and source AMBF as per the instructions on AMBFs wiki: https://github.com/WPI-AIM/ambf/wiki/Installing-AMBF.

## Clone and Build Simulator

### [Recommended] Build with catkin (ROS1)
Since using this with ROS is part of the current intended use case, in these instructions, I assume you will build this in a catkin workspace. Development of all ROS-related items have been compartmentalized to allow for easier updating to ROS2, etc. once that time comes.

These are instructions to build in an existing catkin workspace. If you do not have one yet, take a look at: http://wiki.ros.org/catkin/Tutorials/create_a_workspace

For convenince in setting default filepaths in the code (and in writing these instructions), we suggest users set an environment variable ```CATKIN_WS```. You can check using ``` printenv|grep CATKIN_WS ```. It is recommended to put this directly into your .bashrc so it is set automatically. You only need to do this once!
```bash
echo 'export CATKIN_WS=/home/$USER/bigss/catkin_ws' >> ~/.bashrc
```

# Running the Plugin with ambf_simulator:
The plugin is launched on top of the AMBF simulator along with other AMBF bodies, described by AMBF Description Format files (ADFs), as will be demonstrated below. The `libcontinuum_manip_volumetric_drilling_plugin.so` plugin is initialized in the `launch.yaml` file and can be commented out for the purpose of debugging the ADF files.   

Below are instructions as to how to load different volume and camera options. The -l tag used below allows user to run indexed multibodies that can also be found in the `launch.yaml` under the `multibody configs:` data block. More info on launching the simulator can be found in the AMBF Wiki:  

https://github.com/WPI-AIM/ambf/wiki/Launching-the-Simulator  
https://github.com/WPI-AIM/ambf/wiki/Selecting-Robots  
https://github.com/WPI-AIM/ambf/wiki/Command-Line-Arguments  

Note that the executable binary,`ambf_simulator`, is located in `ambf/bin/lin-x86_64` if you are using Linux. Throughout, some bash scripts may assume you have ```ambf``` installed in ```/home/$USER/```. If you do not, you might need to make a few changes there.

## ROS launch wrapper for ambf executable
There are a few different ways to run ambf, each have their own purposes. Choose your favorite flavor!

### 1. AMBF Native
The original way to run the ambf simulator is to directly call the executable as in, and passing in certain commandline args directly 
```bash
cd ambf/bin/lin-x86_64/
./ambf_simulator --launch_file <continuum-manip-volumetric-drilling-plugin-path>/launch.yaml -l 2,5 --anatomy_volume_name RFemur
```

### 2. Roslaunch commandline
 For convenience, if you want to start the AMBF simulator with this plugin running you can use the ros launch file 
```run_cm_vol_drill_simul.launch```. This launch file has an argument ```ambf_args``` which will be placed in the call to the simulator.

You can use this launch file to replicate the command above directly in the commandline e.g.
``` roslaunch continuum_manip_volumetric_drilling_plugin run_cm_vol_drill_simul.launch ambf_args:="-l 2,5 --anatomy_volume_name RFemur"```

### 3. Roslaunch include
That might seem a bit over-engineered, but this feature was included because you can include it in another launch file as in:
```xml
<include file="$(find continuum_manip_volumetric_drilling_plugin)/launch/run_cm_vol_drill_simul.launch">
    <arg name="ambf_args" value=" \
        -l 2,5 \
        --anatomy_volume_name RFemur"/>
</include>
```

## Running with the continuum manipulator
``` roslaunch continuum_manip_volumetric_drilling_plugin simul_simple_setup.launch```
You can drive the CM using the keyboard. See below for controls and any other info you might need.

You can bring up the CM attached to a UR5 robot using 
``` roslaunch continuum_manup_volumetric_drilling_plugin simul_fullsys_setup.launch```

## Drilling into a different volume
Run the simulator with the added ```--anatomy_volume_name arg``` where arg matches the name given to a volume you are including using the ```-l arg``` command. For example, if there is a volume called ```spine_seg``` that is listed as #15 in the launch.yaml file, and the CM is listed as #25, you could use the following command:
e.g.,
```bash
./ambf_simulator --launch_file $CATKIN_WS/src/continuum-manip-volumetric-drilling-plugin/launch.yaml -l 15,25 --anatomy_volume_name spine_seg
```
The volumes are an array of images (JPG or PNG) that are rendered via texture-based volume rendering. With images and an ADF for the volume, user specified anatomy can easily be used in the simulator. We provide utility scripts (located in the `scripts` folder) that can convert both segmented and non-segmented data from the NRRD format to an array of images.

### Preparing a segmented volume for use in the simulator
#### The new easy way
Use this 3D slicer plugin! https://github.com/htp2/ambf_util_slicer_plugin

#### The old hard way
Make your segmentation using 3D slicer (website: https://www.slicer.org/). Save the segmentation as a NRRD file. Then, use the `scripts/setup_files_for_nrrd_volume.py'. In the commandline, run:
```bash
python3 setup_files_for_nrrd_volume.py -n <path_to_nrrd_file> -v <volume_name> -y <yaml_save_location> -i <png_img_save_location>
```
You can also use ```-p <image_prefix>```  but the default of 'plane0' should work fine

I generally place the yaml_save_location as <plugin-path>/ADF/ and the png_img_save_location as <plugin-path>/resources/volumes/

Upon running, you will need to add the yaml file to your launch.yaml if you want to load that file in using the -l arg.

## Hardness behavior
You can optionally turn on hardness behavior for the volume. This will allow the drill to be affected by the hardness of the material. This is done by using the --hardness_behavior arg and the --hardness_spec_file arg. The --hardness_spec_file arg should be set to the path to the file generated by the `scripts/generate_hardness_file_from_nrrd.py` script. The --hardness_behavior arg should be set to 1 to turn on hardness behavior. Essentially, when the burr is on, contact with a voxel will reduce the harness value, the voxel is removed when the hardness value reaches 0.
### Preparing a file to achieve hardness behavior
Use the `scripts/generate_hardness_file_from_nrrd.py` script to generate a file that will be used to determine the hardness of the volume. In the commandline, run:
```bash 
python3 generate_hardness_file_from_nrrd.py -n <path_to_nrrd_file> -o <output_directory>
```

I generally place the output_directory as <plugin-path>/resources/volumes/[volume_name]/ which will be the same directory as the images generated by the previous script.

This will generate a file called [nrrd_filename]_hardness.csv. This file can be used with the --hardness_spec_file arg and with  --hardness_behavior arg set to 1 to have the simulation use the hardness values in the drilling simulation.

There are plans to improve AMBF readings volume files, so both of these scripts may be deprecated in the future in favor of a more robust built-in solution. 

# Controls
You can interact with the simulator directly with keyboard/mouse commands, or via code (e.g. with ROS sub/pubs). Functionally, control will often be done via ROS, but the keyboard commands are useful for debugging

## Ideosyncracies
1. For now (fix coming): At the start you will need to press: ( Ctrl+] ) and ( Ctrl+[ ) to start the volumetric collisions and have the tool cursors track the mesh positions (this is a workaround to prevent all the tool cursors from starting at 0,0,0 before the first frame and then flying into position, getting stuck and/or causing a bunch of vibrations in the CM).
2. If you are only using keyboard controls, you may need to cycle through switching between the keyboard and topic control modes. This is done by pressing ( Ctrl+/ ) and ( Ctrl+o ) twice. 
3. To use the keyboard controls, the CM needs to be unattached to another AMBF body and needs to be passive (i.e. the base segment should have its mass set to zero). I achieve this by having a [CM_name].ADF and CM_name]_massless.ADF
4. If you attach the CM to another AMBF body, the CM may exhibit strange behavior until topic control is switched on (Ctrl+o) and (Ctrl+/). 

## Keyboard Navigation
1. Control the base of the CM by holding Ctrl and pressing any of the W, A, S, D, I, or K keys for translation. Specifics, and rotation can be found in the table below.

2. Control the bend of the CM by pressing the Ctrl+; and Ctrl+' keys to increase or decrease a 'cable tension' setpoint


| # | Linear Motion of Tool | Description                                  |
|---|-----------------------|----------------------------------------------|
| 1 | [Ctrl+W]              | Moves vertically upward w.r.t. base frame    |
| 2 | [Ctrl+S]              | Moves vertically downward w.r.t. base frame      |
| 3 | [Ctrl+A]              | Moves horizontally left w.r.t. base frame        |
| 4 | [Ctrl+D]              | Moves horizontally right w.r.t. base frame       |
| 5 | [Ctrl+I]              | Moves in the forward direction w.r.t base frame  |
| 6 | [Ctrl+K]              | Moves in the backward direction w.r.t base frame |


| # | Angular Motion of Tool | Description                                     |
|---|------------------------|-------------------------------------------------|
| 1 | [Num 8]                | Rotates towards upward direction w.r.t base frame     |
| 2 | [Num 5]                | Rotates towards downward direction w.r.t. base frame  |
| 3 | [Num 4]                | Rotates towards the left direction w.r.t. base frame  |
| 4 | [Num 6]                | Rotates towards the right direction w.r.t. base frame |


| # | Miscellaneous | Description                                                                        |
|---|---------------|------------------------------------------------------------------------------------|
| 1 | [Ctrl+O (letter o)]      | Toggle the drill's control mode between Haptic Device / Keyboard to ROS Comm       |
| 2 | [Ctrl+N]      | Resets the shape of the volume                                                     |
| 3 | [Alt+R]       | Resets the whole world and this plugin                                             |
| 4 | [Ctrl+C] | Toggles the visbility of collision spheres |
| 7 | [Ctrl+[ ] | Toggles tip volume collision |
| 8 | [Ctrl+] ] | Resets collision spheres with mesh locations |
| 9 | [Ctrl+/ ] | Toggles whether cable setting comes from keyboard or ROS topic |
| 10 | [Ctrl+; ] | Decreases cable pull magnitude |
| 11 | [Ctrl+' ] | Increases cable pull magnitude |
| 12 | [Ctrl+= ] | Toggles the burr as on/off (whether or not voxels will be removed from anatomy) |
## 2.5.2 Mouse Movement
Navigation using mouse shortcuts in AMBF is described here: https://github.com/WPI-AIM/ambf/wiki/Keyboard-and-Mouse-Shortcuts
## Several Settings and Options Available to use as ROS Topics
Several of the keyboard commands have been supplemented with ros topics that will carry out the same features. 

A list is below. All topics accept std_msgs::Bool. The namespace is ```/ambf/volumetric_drilling/```

| # | Topic Name | Description                                  |
|---|-----------------------|----------------------------------------------|
| 1 | setShowToolCursors         | Sets whether tool cursors will be rendered as sphere (true=rendered)   |
| 2 | setDrillControlMode             | Sets drill's control mode between Haptic Device / Keyboard and ROS Comm (true=ROS)    |
| 3 | setVolumeCollisionsEnabled              | Sets if volume collision occurs (true=collision)         |
| 4 | setCableControlMode              | Sets cables's control mode between keyboard and ROS Comm (true=Keyboard)      |
| 5 | setPhysicsPaused              | Sets if simulator is paused (true=pause)  |
| 6 | initToolCursors              | Resets collision spheres with mesh locations (true or false will trigger this) |
| 7 | resetVoxels | Resets volume, any removed voxels are returned (true or false will trigger this)
| 8 | setBurrOn | Sets if burr is on, i.e. if burr collision removes voxels (true=on)

It is fairly straightforward to set up pubs for these in e.g. high-level python control scripts
