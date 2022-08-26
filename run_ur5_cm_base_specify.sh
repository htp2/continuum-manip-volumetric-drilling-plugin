#!/bin/bash

/home/henry/ambf/bin/lin-x86_64/ambf_simulator \
    --launch_file /home/henry/bigss/catkin_ws/src/continuum-manip-volumetric-drilling-plugin/launch.yaml \
    -l 15,29,31 \
    --name_body_to_trace Burr \
    --anatomy_volume_name spine_seg \
    --csv_filename_static_traces /home/henry/bigss/catkin_ws/src/continuum-manip-volumetric-drilling-plugin/resources/axis.csv \
    --body_base_attached_to_name wrist_3_link
