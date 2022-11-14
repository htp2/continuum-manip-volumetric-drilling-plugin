#!/bin/bash

cam=$1
slice=$2
id=$3
ambf_path=$4
rob_path=$5
outdir=$6

echo "c: $cam, s: $slice, id: $id" 
roslaunch continuum_manip_volumetric_drilling_plugin ambf_curved_drilling_ipcai2022TEST2.launch constraint_config:=${rob_path}${id}_constraints_config.json trace_path:=${ambf_path}${id}.csv &
pid=$!            
sleep 5
timeout -k 10 10m python3 /home/henry/bigss/catkin_ws/src/bigss_spine/spine_python/ambf_highlevel_ipcai2022_script_traj_select.py -t ${rob_path}${id}_traj.csv -o $outdir -e $cam -s $slice --no-stamp 
kill $pid
# pkill -f "AMBF" -9
