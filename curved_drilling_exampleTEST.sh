#!/bin/bash

filename=$1

/home/$USER/ambf/bin/lin-x86_64/ambf_simulator \
    --launch_file $CATKIN_WS/src/continuum-manip-volumetric-drilling-plugin/launch.yaml \
    -l 11,15,25,29,30 \
    --name_body_to_trace seg27 \
    --anatomy_volume_name spine_seg \
    --csv_filename_static_traces $filename \
    --static_trace_rel_body_name spine_seg \
    --vary_drilling_behavior 1 \
    --debug_traj_file $filename 

