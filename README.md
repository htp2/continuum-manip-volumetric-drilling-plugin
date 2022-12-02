This repo was developed by Henry Phalen as part of his work as a graduate student at Johns Hopkins University.  The program simulates a continuum manipulator that is able to manipulate / remove parts of a volumetric model. The initial application is to simulate the control of a dexterous surgical tool for curved drilling for autonomous surgical procedures in the femur and spine. This is implemented as a plugin for AMBF https://github.com/WPI-AIM/ambf/

Adapted from the work of Adnan Munawar et al. See their work at https://github.com/LCSR-SICKKIDS/volumetric_drilling

This plugin is an 'unofficial fork' of that repository. Both have undergone significant development since the split so there is some divergence. The plan is to converge at least the volumetric drilling (i.e. what happens at the burr) at some point.

This pairs well with another plugin I wrote (https://github.com/htp2/ambf_trace_plugin), you might find reference to it in the launch file!

# Installation Instructions:
## Install and Source AMBF 2.0

Build and source AMBF as per the instructions on AMBFs wiki: https://github.com/WPI-AIM/ambf/wiki/Installing-AMBF.

## Clone and Build Simulator

### [Recommended] Build with catkin (ROS1)
Since using this with ROS is part of the current intended use case, in these instructions, I assume you will build this in a catkin workspace. It is likely possible to build this repo without catkin (without ROS). Further development of all ROS-related items have been compartmentalized to allow for easier updating to ROS2, etc. once that time comes.

These are instructions to build in an existing catkin workspace. If you do not have one yet, take a look at: http://wiki.ros.org/catkin/Tutorials/create_a_workspace

For convenince in setting default filepaths in the code (and in writing these instructions), we suggest users set an environment variable ```CATKIN_WS```. You can check using ``` printenv|grep CATKIN_WS ```. It is recommended to put this directly into your .bashrc so it is set automatically. You only need to do this once!
```bash
echo 'export CATKIN_WS=/home/$USER/bigss/catkin_ws' >> ~/.bashrc
```

# Running the Plugin with ambf_simulator:
The volumetric drilling simulator is a plugin that is launched on top of the AMBF simulator along with other AMBF bodies, described by AMBF Description Format files (ADFs), as will be demonstrated below. The `libcontinuum_manip_volumetric_drilling_plugin.so` plugin is initialized in the `launch.yaml` file and can be commented out for the purpose of debugging the ADF files.   

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
./ambf_simulator --launch_file <continuum-manip-volumetric-drilling-plugin-path>/launch.yaml -l 24,25 --anatomy_volume_name cube
```

### 2. Roslaunch commandline
 For convenience, if you want to start the AMBF simulator with this plugin running you can use the ros launch file 
```run_cm_vol_drill_simul.launch```. This launch file has an argument ```ambf_args``` which will be placed in the call to the simulator.

You can use this launch file to replicate the command above directly in the commandline e.g.
``` roslaunch continuum_manip_volumetric_drilling_plugin run_cm_vol_drill_simul.launch ambf_args:="-l 24,25 --anatomy_volume_name cube"```

### 3. Roslaunch include
That might seem a bit over-engineered, but this feature was included because you can include it in another launch file as in:
```xml
<include file="$(find continuum_manip_volumetric_drilling_plugin)/launch/run_cm_vol_drill_simul.launch">
    <arg name="ambf_args" value=" \
        -l 24,25 \
        --anatomy_volume_name cube"/>
</include>
```

## Running with the continuum manipulator
[TODO]: Put a simple example here

## Drilling into a different volume
Run the simulator with the added ```--anatomy_volume_name arg``` where arg matches the name given to a volume you are including using the ```-l arg``` command. For example, if there is a volume called ```spine_seg``` that is listed as #15 in the launch.yaml file, and the CM is listed as #25, you could use the following command:
e.g.,
```bash
./ambf_simulator --launch_file $CATKIN_WS/src/continuum-manip-volumetric-drilling-plugin/launch.yaml -l 15,25 --anatomy_volume_name spine_seg
```
The volumes are an array of images (JPG or PNG) that are rendered via texture-based volume rendering. With images and an ADF for the volume, user specified anatomy can easily be used in the simulator. We provide utility scripts (located in the `scripts` folder) that can convert both segmented and non-segmented data from the NRRD format to an array of images.
# Controls
You can interact with the simulator directly with keyboard/mouse commands, or via code (e.g. with ROS sub/pubs). Functionally, control will often be done via ROS, but the keyboard commands are useful for debugging

## Ideosyncracies
1. For now (fix coming): At the start you will need to press: ( Ctrl+] ) and ( Ctrl+[ ) to start the volumetric collisions and have the tool cursors track the mesh positions (this is a workaround to prevent all the tool cursors from starting at 0,0,0 before the first frame and then flying into position, getting stuck and/or causing a bunch of vibrations in the CM).

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
| 3 | [B]           | Toggles the visibility of drill mesh in the scene                                  |
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