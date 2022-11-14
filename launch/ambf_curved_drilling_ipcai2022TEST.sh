#!/bin/bash


run_function()
{
    for cam in 0 1 2 3 4 5
    do
        for slice in 0 1 2 3 4 5

            
            python3 /home/henry/bigss/catkin_ws/src/bigss_spine/spine_python/ambf_highlevel_ipcai2022_script.py -t /home/henry/bigss/catkin_ws/src/bigss_spine/spine_robot_control/config/sim_ur5/points.txt -o /home/henry/exps/20221102_ipcai22_firstpass -e $cam -s $slice
            


        done
    done

}

launch()
{
    roslaunch continuum_manip_volumetric_drilling_plugin ambf_curved_drilling_ipcai2022TEST.launch
}

xreg-snake-htp-imager-ros-main "/home/$USER/snake_registration/simulation/JustinSnakeModel_huatt/ /home/$USER/snake_registration/simulation/output/ /home/$USER/snake_registration/simulation/anatomy/" &&
rosrun rqt_image_view rqt_image_view &&






<node pkg="continuum_manip_volumetric_drilling_plugin" name="ambf_with_CM_plugin" type="curved_drilling_example.sh"
    output="screen"/>

<node pkg="continuum_manip_volumetric_drilling_plugin" name="ambf_ros_ur5" type="ur5_ambf.py" output="screen"/> 



<include file="$(find snake_estimator)/launch/snake_estimator_kalman.launch"/> 

<include file="$(find spine_robot_control)/launch/URWithSnakeAMBF.launch"/> 


<arg name="node_start_delay" default="5.0" />  
<node pkg="spine_python" type="ambf_highlevel_ipcai2022_script.py" name="ambf_highlevel_ipcai2022_script"  output="screen"
args="-t $(find spine_robot_control)/config/sim_ur5/points.txt -o /home/henry/exps/20221102_ipcai22_firstpass -e 0 -s 0"
launch-prefix="bash -c 'sleep $(arg node_start_delay); $0 $@' " />    <!-- This delays the high-level script for a bit to give everything a chance to start up -->





 <arg name="node_start_delay" default="5.0" />  
  <node pkg="spine_python" type="ambf_highlevel_ipcai2022_script.py" name="ambf_highlevel_ipcai2022_script"  output="screen"
    args="-t $(find spine_robot_control)/config/sim_ur5/points.txt -o /home/henry/exps/20221102_ipcai22_firstpass -e 0 -s 0"
    launch-prefix="bash -c 'sleep $(arg node_start_delay); $0 $@' " />    <!-- This delays the high-level script for a bit to give everything a chance to start up -->
