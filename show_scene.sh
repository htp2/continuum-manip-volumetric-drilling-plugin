#!/bin/bash

/home/$USER/ambf/bin/lin-x86_64/ambf_simulator \
  --launch_file $CATKIN_WS/src/continuum-manip-volumetric-drilling-plugin/launch_scene.yaml \
  -l 11,25,36,37,38,39,30 \
  --name_body_to_trace seg27 \
  --csv_filename_static_traces $CATKIN_WS/src/continuum-manip-volumetric-drilling-plugin/ADF/spine/pt.csv \
  --anatomy_volume_name spine_seg \
  --static_trace_rel_body_name spine_seg
