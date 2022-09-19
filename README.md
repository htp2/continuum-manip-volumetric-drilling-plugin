Adapted from the work of Adnan Munawar et al. See their work at https://github.com/LCSR-SICKKIDS/volumetric_drilling

This repo was developed by Henry Phalen as part of his work as a graduate student at Johns Hopkins University. The program simulates a continuum manipulator that is able to manipulate / remove parts of a volumetric model.

The initial application is to simulate the control of a dexterous surgical tool for curved drilling for autonomous surgical procedures in the femur and spine.

Most of the instructions below are the same as https://github.com/LCSR-SICKKIDS/volumetric_drilling (Thanks Adnan!), but there are some additions, particularly in the keyboard hotkey section.


This pairs well with another plugin I wrote (https://github.com/htp2/ambf_trace_plugin), you might find reference to it in the launch file!


## 1. Installation Instructions:

### 1.1 Install and Source AMBF 2.0

Build and source ambf (make sure you're on branch ambf-2.0 before building) as per the instructions on AMBFs wiki: https://github.com/WPI-AIM/ambf/wiki/Installing-AMBF.

#### Functionality supported by volumetric-drilling
I (Henry) haven't used this, but it should work just fine, so I'm copying the relevant instructions from ```volumetric_drilling``` below:

>Note that depth and image recording are enabled by default (in camera ADFs) and these features only work on Linux with ROS installed. Additionally, the following packages must be installed prior to building to AMBF:

```bash
cv-bridge # Can be installed via apt install ros-<version>-cv-bridge
image-transport # Can be installed via apt install ros-<version>-image-transport
```

### 1.2 Clone and Build Simulator

#### [Recommended] Build with catkin
These are instructions to build in an existing catkin workspace. If you do not have one yet, take a look at: http://wiki.ros.org/catkin/Tutorials/create_a_workspace

For convenince in setting default filepaths in the code (and in writing these instructions), we suggest users set an environment variable ```CATKIN_WS``` 

If you have not set CATKIN_WS as an environment variable, do so now. You can check using ``` printenv|grep CATKIN_WS ```. It is recommended to put this directly into your .bashrc so it is set automatically. You only need to do this once!
```bash
echo 'export CATKIN_WS=/home/henry/bigss/catkin_ws' >> ~/.bashrc
```
##### Clone and build
``` bash
cd $CATKIN_WS
git clone <this repo>
catkin build
```
#### Building outside catkin
You can also just build using CMake
``` bash
git clone <this repo>
cd <continuum-manip-volumetric-drilling-plugin-path>
mkdir build
cd build
cmake ..
make
```

## 2 Running the Plugin with ambf_simulator:
The volumetric drilling simulator is a plugin that is launched on top of the AMBF simulator along with other AMBF bodies, described by AMBF Description Format files (ADFs), as will be demonstrated below. The `libcontinuum_manip_volumetric_drilling_plugin.so` plugin is initialized in the `launch.yaml` file and can be commented out for the purpose of debugging the ADF files.   

Below are instructions as to how to load different volume and camera options. The -l tag used below allows user to run indexed multibodies that can also be found in the `launch.yaml` under the `multibody configs:` data block. More info on launching the simulator can be found in the AMBF Wiki:  

https://github.com/WPI-AIM/ambf/wiki/Launching-the-Simulator  
https://github.com/WPI-AIM/ambf/wiki/Selecting-Robots  
https://github.com/WPI-AIM/ambf/wiki/Command-Line-Arguments  

Note that the executable binary,`ambf_simulator`, is located in `ambf/bin/lin-x86_64` if you are using Linux. Throughout, some bash scripts may assume you have ```ambf``` installed in ```/home/$USER/```. If you do not, you might need to make a few changes there.

### Running with the continuum manipulator
To startup with a simple block volume you can drill into, you can use:
```bash
cd ambf/bin/lin-x86_64/
./ambf_simulator --launch_file <continuum-manip-volumetric-drilling-plugin-path>/launch.yaml -l 24,25
```
### Drilling into a different volume
Run the simulator with the added ```--anatomy_volume_name arg``` where arg matches the name given to a volume you are including using the ```-l arg``` command. For example, if there is a volume called ```spine_seg``` that is listed as #15 in the launch.yaml file, and the CM is listed as #25, you could use the following command:
e.g.,
```bash
./ambf_simulator --launch_file $CATKIN_WS/src/continuum-manip-volumetric-drilling-plugin/launch.yaml -l 15,25 --anatomy_volume_name spine_seg
```
#### Controls with continuum manipulator (CM)
1. At the start you will need to press: ( Ctrl+] ) and ( Ctrl+[ ) to start the volumetric collisions and have the tool cursors track the mesh positions (this is a workaround to prevent all the tool cursors from starting at 0,0,0 before the first frame and then flying into position, getting stuck and/or causing a bunch of vibrations in the CM). This will be fixed eventually.

2. Control the base of the CM by holding Ctrl and pressing any of the W, A, S, D, I, or K keys for translation. Specifics, and rotation can be found in the table below.

3. Control the bend of the CM by pressing the Ctrl+; and Ctrl+' keys to increase or decrease a 'cable tension' setpoint

#### Several Settings and Options Available to use as ROS Topics
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

#### Option 4: User-provided volume
Patient specific anatomy may also be used in the simulator. The volumes are an array of images (JPG or PNG) that are rendered via texture-based volume rendering. With images and an ADF for the volume, user specified anatomy can easily be used in the simulator. We provide utility scripts (located in the `scripts` folder) that can convert both segmented and non-segmented data from the NRRD format to an array of images.


### 2.3 Changing Scene Parameters
All the relevant ADF scene objects are in the ADF folder and can be modified as needed. For example, camera intrinsics can be adjusted via the field view angle and image resolution parameters of Camera ADFs.

### 2.4 Manipulating Drill
The virtual drill can be manipulated via a keyboard or using ROS commands

#### 2.4.1 Keyboard Navigation

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


### 2.5 Navigating in Simulator
Camera movement in the simulator can be accomplished through AMBF's python client, mouse movement or Head Mounted Displays (HMDs)
#### 2.5.1 AMBF Python Client
Camera can be moved with the AMBF python client as described here: https://github.com/WPI-AIM/ambf/wiki/The-Python-Client. To move all cameras in sync the object that should be moved is the parent of all the cameras, `main_camera`.  
Note that only one instance of the AMBF python client can be opened at a time. The data generation script uses the python client, hence camera movement must be added to that script if data is also being recorded.

#### 2.5.2 Mouse Movement
Navigation using mouse shortcuts in AMBF is described here: https://github.com/WPI-AIM/ambf/wiki/Keyboard-and-Mouse-Shortcuts


### 2.6 Data Recording

A note from Henry: I don't use this, but I am leaving the instructions below!

>A python script (`scripts/data_record.py`) is provided to record data based on the user's configuration. By default, the left and right stereo images, depth point cloud, segmentation mask,drill/camera poses, removed voxels and drill burr changes are recorded. The data is stored as a convenient and well-organized hdf5 file.
NOTE: 
- Source the ambf and vdrilling_msgs environment in terminal before running the script.
- By default, data recording should be launched after the simulator. We perform sanity check on this to make sure topics subscribed are meaningful.
