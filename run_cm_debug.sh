#!/bin/bash

/home/henry/ambf/bin/lin-x86_64/ambf_simulator \
    --launch_file /home/henry/bigss/catkin_ws/src/continuum-manip-volumetric-drilling-plugin/launch.yaml \
    -l 11,34,33,19,20 \
    --name_body_to_trace Burr \
    --anatomy_volume_name spine_seg